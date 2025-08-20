set(srcs 
    "sx126x_bsp.c"
    "radio.c"
    "sx126x.c"
    "timer.c")

idf_component_register(
    SRCS "${srcs}"
    INCLUDE_DIRS .
    REQUIRES
        freertos
        driver
        esp_timer
        esp_driver_gpio)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")
