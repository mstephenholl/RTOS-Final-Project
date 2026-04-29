idf_component_register(
    SRCS
        "sx1262.c"
        "sx1262_hal.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES driver freertos esp_timer log
)
