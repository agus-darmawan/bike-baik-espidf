idf_component_register(
    SRCS 
        "mpu6050.c" 
        "moving_average.c" 
        "mahony.c" 
        "madgwick.c"
    INCLUDE_DIRS "include"
    REQUIRES 
        driver 
        esp_timer
)