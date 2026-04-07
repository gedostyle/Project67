idf_component_register(SRCS "main.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES vl53l0x esp_driver_uart esp_driver_gpio esp_driver_i2c)