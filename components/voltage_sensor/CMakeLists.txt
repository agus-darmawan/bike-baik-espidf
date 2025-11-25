idf_component_register(
    SRCS "voltage_sensor.c"
    INCLUDE_DIRS "include"
    REQUIRES 
        esp_adc
        esp_driver_gpio
)