idf_component_register(
    SRCS "max6675.c"
    INCLUDE_DIRS "include"
    REQUIRES    
        esp_driver_spi    
        esp_driver_gpio
)
                    
