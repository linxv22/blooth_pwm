idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES ble_server nvs_flash motor_driver adc_driver)
