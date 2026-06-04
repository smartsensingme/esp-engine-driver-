idf_component_register(SRCS "engine_driver.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_mcpwm esp_driver_gpio)
