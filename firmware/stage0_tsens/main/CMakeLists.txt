idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_driver_gpio
                             esp_driver_tsens
                             esp_driver_rmt
                             esp_timer
                             esp_common)
