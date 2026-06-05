idf_component_register(
    SRCS
        # sensor_hal is header-only; no .c file needed
    INCLUDE_DIRS
        "include"
    REQUIRES
        driver
        log
)
