idf_component_register(
    SRCS 
        "sim808.c"
        "sim808_http.c"
        "sim808_mqtt.c"
    INCLUDE_DIRS "include"
    REQUIRES 
        driver
        esp_timer
)
