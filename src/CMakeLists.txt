idf_component_register(
    SRCS        "main.c"
    INCLUDE_DIRS "."
    REQUIRES    sx1262 ssd1306 driver freertos esp_timer log esp_hw_support
)
