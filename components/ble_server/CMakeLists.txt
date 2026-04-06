# 编译当前目录下的 ble_server.c，并且把当前目录 "." 暴露给外部当做头文件搜索路径
# 使用Bluedroid蓝牙堆栈
idf_component_register(SRCS "ble_server.c" 
                       INCLUDE_DIRS "."
                       REQUIRES bt motor_driver adc_driver )