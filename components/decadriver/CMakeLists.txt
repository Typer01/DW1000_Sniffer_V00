idf_component_register(
    SRCS
        "deca_device.c"
        "deca_params_init.c"
        "deca_sleep.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_driver_gpio 
        freertos
        


)
