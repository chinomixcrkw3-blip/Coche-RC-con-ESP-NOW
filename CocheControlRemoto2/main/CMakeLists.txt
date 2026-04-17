idf_component_register(SRCS "CocheControlRemoto2.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi nvs_flash esp_event esp_netif esp_driver_gpio esp_driver_ledc)
