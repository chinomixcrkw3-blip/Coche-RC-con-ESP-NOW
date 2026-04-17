#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

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
#define SEND_PERIOD_MS          20

// MAC DEL RECEPTOR (CAMBIA ESTA MAC)
static const uint8_t RX_MAC[ESP_NOW_ETH_ALEN] = { 0x00, 0x00, 0x00, 0xDD, 0xDD, 0xDD };

// =====================================================
// JOYSTICK
// =====================================================
#define JOY_X_GPIO              GPIO_NUM_5      // ADC1_CH0
#define JOY_Y_GPIO              GPIO_NUM_4      // ADC1_CH1
#define JOY_SW_GPIO             GPIO_NUM_6

#define JOY_X_CH                ADC_CHANNEL_4
#define JOY_Y_CH                ADC_CHANNEL_3

#define JOY_X_CENTER            2048
#define JOY_Y_CENTER            2048
#define JOY_DEADZONE            220
#define JOY_Y_INVERT            1               // 1 = empujar hacia adelante da positivo

// =====================================================
// PAQUETE
// =====================================================
#define PKT_MAGIC               0xA55A1234

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
static const char *TAG = "TX_ESPNOW";
static adc_oneshot_unit_handle_t s_adc1;
static volatile bool s_tx_ready = true;
static uint32_t s_seq = 0;

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
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));

    print_mac_sta();
}

static void on_espnow_send(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    (void)tx_info;

    s_tx_ready = true;

    if (status != ESP_NOW_SEND_SUCCESS) {
        ESP_LOGW(TAG, "Fallo al enviar paquete");
    }
}

static void espnow_init_sender(void)
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(on_espnow_send));

    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, RX_MAC, ESP_NOW_ETH_ALEN);
    peer.ifidx = WIFI_IF_STA;
    peer.channel = WIFI_CHANNEL;
    peer.encrypt = false;

    ESP_ERROR_CHECK(esp_now_add_peer(&peer));
}

static void joystick_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &s_adc1));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };

    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1, JOY_X_CH, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc1, JOY_Y_CH, &chan_cfg));

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << JOY_SW_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
}

static int read_adc_avg(adc_channel_t ch)
{
    int raw = 0;
    int sum = 0;

    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(adc_oneshot_read(s_adc1, ch, &raw));
        sum += raw;
    }

    return sum / 4;
}

static int normalize_axis(int raw, int center)
{
    int delta = raw - center;

    if (delta > -JOY_DEADZONE && delta < JOY_DEADZONE) {
        return 0;
    }

    if (delta > 0) {
        int num = (delta - JOY_DEADZONE) * 100;
        int den = (4095 - center - JOY_DEADZONE);
        int out = (den > 0) ? (num / den) : 0;
        if (out > 100) out = 100;
        return out;
    } else {
        int num = (delta + JOY_DEADZONE) * 100;
        int den = (center - JOY_DEADZONE);
        int out = (den > 0) ? (num / den) : 0;
        if (out < -100) out = -100;
        return out;
    }
}

void app_main(void)
{
    nvs_init_safe();
    wifi_init_sta_fixed_channel();
    espnow_init_sender();
    joystick_init();

    ESP_LOGI(TAG, "Transmisor listo");

    while (1) {
        int raw_x = read_adc_avg(JOY_X_CH);
        int raw_y = read_adc_avg(JOY_Y_CH);

        int steer = normalize_axis(raw_x, JOY_X_CENTER);
        int throttle = normalize_axis(raw_y, JOY_Y_CENTER);

#if JOY_Y_INVERT
        throttle = -throttle;
#endif

        uint8_t sw = (gpio_get_level(JOY_SW_GPIO) == 0) ? 1 : 0;

        control_packet_t pkt = {
            .magic = PKT_MAGIC,
            .steer = steer,
            .throttle = throttle,
            .sw = sw,
            .seq = s_seq++,
        };

        if (s_tx_ready) {
            s_tx_ready = false;

            esp_err_t err = esp_now_send(RX_MAC, (const uint8_t *)&pkt, sizeof(pkt));
            if (err != ESP_OK) {
                s_tx_ready = true;
                ESP_LOGW(TAG, "esp_now_send error: %s", esp_err_to_name(err));
            }
        }

        ESP_LOGI(TAG, "rawX=%4d rawY=%4d steer=%4d thr=%4d sw=%d",
                 raw_x, raw_y, steer, throttle, sw);

        vTaskDelay(pdMS_TO_TICKS(SEND_PERIOD_MS));
    }
}