idf_component_register(
    SRCS        "ssd1306.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "."
    REQUIRES    driver freertos log
)
