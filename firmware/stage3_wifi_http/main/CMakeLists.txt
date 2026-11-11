idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_wifi
                             esp_event
                             esp_netif
                             nvs_flash
                             esp_http_server
                             esp_timer
                             esp_driver_tsens
                             esp_common)
