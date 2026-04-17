#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

// =====================================================
// CONFIGURACIÓN GENERAL
// =====================================================
#define WIFI_CHANNEL            1
#define PKT_MAGIC               0xA55A1234
#define RX_TIMEOUT_MS           300

// =====================================================
// SERVO
// =====================================================
#define SERVO_GPIO              GPIO_NUM_7

#define SERVO_MODE              LEDC_LOW_SPEED_MODE
#define SERVO_TIMER             LEDC_TIMER_0
#define SERVO_CHANNEL           LEDC_CHANNEL_0
#define SERVO_FREQUENCY_HZ      50
#define SERVO_RESOLUTION        LEDC_TIMER_14_BIT
#define SERVO_DUTY_BITS         14

#define SERVO_PERIOD_US         20000
#define SERVO_MIN_PULSE_US      500
#define SERVO_MAX_PULSE_US      2500

#define SERVO_IZQUIERDA_DEG     80
#define SERVO_CENTRO_DEG        94
#define SERVO_DERECHA_DEG       108

#define SERVO_STEP_DEG          1
#define SERVO_TASK_MS           20

#define MOTOR_PWM_GPIO          GPIO_NUM_4
#define MOTOR_IN1_GPIO          GPIO_NUM_5
#define MOTOR_IN2_GPIO          GPIO_NUM_6

#define MOTOR_MODE              LEDC_LOW_SPEED_MODE
#define MOTOR_TIMER             LEDC_TIMER_1
#define MOTOR_CHANNEL           LEDC_CHANNEL_1
#define MOTOR_FREQUENCY_HZ      20000
#define MOTOR_RESOLUTION        LEDC_TIMER_11_BIT
#define MOTOR_DUTY_BITS         11
#define MOTOR_MAX_DUTY          ((1U << MOTOR_DUTY_BITS) - 1U)
#define MOTOR_DEADZONE          8

// =====================================================
// PAQUETE
// =====================================================
typedef struct __attribute__((packed)) {
    uint32_t magic;
    int16_t steer;      // -100 a 100
    int16_t throttle;   // -100 a 100
    uint8_t sw;         // 0 o 1
    uint32_t seq;
} control_packet_t;

// =====================================================
// GLOBALES
// =====================================================
static const char *TAG = "RX_ESPNOW";
static QueueHandle_t s_rx_queue = NULL;

static volatile int servo_target_deg = SERVO_CENTRO_DEG;
static int servo_current_deg = SERVO_CENTRO_DEG;
static TickType_t s_last_rx_tick = 0;

// =====================================================
// UTILIDADES
// =====================================================
static void print_mac_sta(void)
{
    uint8_t mac[6];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
    ESP_LOGI(TAG, "MAC STA local: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void nvs_init_safe(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK(err);
}

static void wifi_init_sta_fixed_channel(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));

    print_mac_sta();
}

// =====================================================
// SERVO
// =====================================================
static uint32_t servo_us_to_duty(uint32_t pulse_us)
{
    uint32_t max_duty = (1U << SERVO_DUTY_BITS) - 1U;
    return (pulse_us * max_duty) / SERVO_PERIOD_US;
}

static uint32_t servo_deg_to_us(int angle_deg)
{
    if (angle_deg < 0) angle_deg = 0;
    if (angle_deg > 180) angle_deg = 180;

    return SERVO_MIN_PULSE_US +
           ((SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * angle_deg) / 180;
}

static esp_err_t servo_apply_angle(int angle_deg)
{
    uint32_t pulse_us = servo_deg_to_us(angle_deg);
    uint32_t duty = servo_us_to_duty(pulse_us);

    ESP_ERROR_CHECK(ledc_set_duty(SERVO_MODE, SERVO_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(SERVO_MODE, SERVO_CHANNEL));

    return ESP_OK;
}

static esp_err_t servo_init(void)
{
    ledc_timer_config_t timer_config = {
        .speed_mode       = SERVO_MODE,
        .duty_resolution  = SERVO_RESOLUTION,
        .timer_num        = SERVO_TIMER,
        .freq_hz          = SERVO_FREQUENCY_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t channel_config = {
        .gpio_num       = SERVO_GPIO,
        .speed_mode     = SERVO_MODE,
        .channel        = SERVO_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = SERVO_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

    servo_current_deg = SERVO_CENTRO_DEG;
    servo_target_deg = SERVO_CENTRO_DEG;
    ESP_ERROR_CHECK(servo_apply_angle(servo_current_deg));

    return ESP_OK;
}

static void direccion_set_grados(int grados)
{
    if (grados < SERVO_IZQUIERDA_DEG) grados = SERVO_IZQUIERDA_DEG;
    if (grados > SERVO_DERECHA_DEG) grados = SERVO_DERECHA_DEG;

    servo_target_deg = grados;
}

static void servo_task(void *pvParameters)
{
    while (1) {
        if (servo_current_deg < servo_target_deg) {
            servo_current_deg += SERVO_STEP_DEG;
            if (servo_current_deg > servo_target_deg) {
                servo_current_deg = servo_target_deg;
            }
            ESP_ERROR_CHECK(servo_apply_angle(servo_current_deg));
        } else if (servo_current_deg > servo_target_deg) {
            servo_current_deg -= SERVO_STEP_DEG;
            if (servo_current_deg < servo_target_deg) {
                servo_current_deg = servo_target_deg;
            }
            ESP_ERROR_CHECK(servo_apply_angle(servo_current_deg));
        }

        vTaskDelay(pdMS_TO_TICKS(SERVO_TASK_MS));
    }
}

// =====================================================
// MOTOR
// =====================================================
static esp_err_t motor_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_IN1_GPIO) | (1ULL << MOTOR_IN2_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ledc_timer_config_t timer_config = {
        .speed_mode       = MOTOR_MODE,
        .duty_resolution  = MOTOR_RESOLUTION,
        .timer_num        = MOTOR_TIMER,
        .freq_hz          = MOTOR_FREQUENCY_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

    ledc_channel_config_t channel_config = {
        .gpio_num       = MOTOR_PWM_GPIO,
        .speed_mode     = MOTOR_MODE,
        .channel        = MOTOR_CHANNEL,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = MOTOR_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));

    gpio_set_level(MOTOR_IN1_GPIO, 0);
    gpio_set_level(MOTOR_IN2_GPIO, 0);
    ESP_ERROR_CHECK(ledc_set_duty(MOTOR_MODE, MOTOR_CHANNEL, 0));
    ESP_ERROR_CHECK(ledc_update_duty(MOTOR_MODE, MOTOR_CHANNEL));

    return ESP_OK;
}

static void motor_set(int throttle_pct)
{
    if (throttle_pct > 100) throttle_pct = 100;
    if (throttle_pct < -100) throttle_pct = -100;

    int abs_pct = abs(throttle_pct);
    uint32_t duty = (abs_pct * MOTOR_MAX_DUTY) / 100U;

    if (abs_pct <= MOTOR_DEADZONE) {
        gpio_set_level(MOTOR_IN1_GPIO, 0);
        gpio_set_level(MOTOR_IN2_GPIO, 0);
        duty = 0;
    } else if (throttle_pct > 0) {
        gpio_set_level(MOTOR_IN1_GPIO, 1);
        gpio_set_level(MOTOR_IN2_GPIO, 0);
    } else {
        gpio_set_level(MOTOR_IN1_GPIO, 0);
        gpio_set_level(MOTOR_IN2_GPIO, 1);
    }

    ESP_ERROR_CHECK(ledc_set_duty(MOTOR_MODE, MOTOR_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(MOTOR_MODE, MOTOR_CHANNEL));
}

// =====================================================
// ESP-NOW RX
// =====================================================
static void on_espnow_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    (void)info;

    if (len != sizeof(control_packet_t)) {
        return;
    }

    control_packet_t pkt;
    memcpy(&pkt, data, sizeof(pkt));

    if (pkt.magic != PKT_MAGIC) {
        return;
    }

    if (s_rx_queue != NULL) {
        xQueueOverwrite(s_rx_queue, &pkt);
    }
}

static void espnow_init_receiver(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(on_espnow_recv));
}

// =====================================================
// CONTROL PRINCIPAL
// =====================================================
static int map_steer_to_angle(int steer_pct)
{
    if (steer_pct > 100) steer_pct = 100;
    if (steer_pct < -100) steer_pct = -100;

    // Mapea linealmente -100..100 a 80..108
    int angle = SERVO_IZQUIERDA_DEG +
                ((steer_pct + 100) * (SERVO_DERECHA_DEG - SERVO_IZQUIERDA_DEG)) / 200;

    return angle;
}

static void control_task(void *pvParameters)
{
    control_packet_t pkt;

    s_last_rx_tick = xTaskGetTickCount();

    while (1) {
        if (xQueueReceive(s_rx_queue, &pkt, pdMS_TO_TICKS(50)) == pdTRUE) {
            s_last_rx_tick = xTaskGetTickCount();

            int angle = map_steer_to_angle(pkt.steer);
            direccion_set_grados(angle);
            motor_set(pkt.throttle);

            ESP_LOGI(TAG, "seq=%" PRIu32 " steer=%d throttle=%d sw=%d angle=%d",
                     pkt.seq, pkt.steer, pkt.throttle, pkt.sw, angle);
        }

        TickType_t now = xTaskGetTickCount();
        if ((now - s_last_rx_tick) > pdMS_TO_TICKS(RX_TIMEOUT_MS)) {
            direccion_set_grados(SERVO_CENTRO_DEG);
            motor_set(0);
        }
    }
}

void app_main(void)
{
    nvs_init_safe();
    wifi_init_sta_fixed_channel();

    s_rx_queue = xQueueCreate(1, sizeof(control_packet_t));
    if (s_rx_queue == NULL) {
        ESP_LOGE(TAG, "No se pudo crear la cola");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    ESP_ERROR_CHECK(servo_init());
    ESP_ERROR_CHECK(motor_init());
    espnow_init_receiver();

    xTaskCreate(servo_task, "servo_task", 2048, NULL, 5, NULL);
    xTaskCreate(control_task, "control_task", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Receptor listo");
}