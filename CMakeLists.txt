idf_component_register(SRCS "engine_driver.c"
                            "engine_current_sense.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_adc
                                esp_driver_mcpwm
                                esp_driver_gpio
                                freertos)
