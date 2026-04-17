idf_component_register(SRCS "ESP32_Control.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi nvs_flash esp_event esp_netif esp_adc esp_driver_gpio)
