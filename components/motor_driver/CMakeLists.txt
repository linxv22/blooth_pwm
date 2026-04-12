idf_component_register(SRCS "motor_driver.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_ledc esp_driver_gpio adc_driver)
