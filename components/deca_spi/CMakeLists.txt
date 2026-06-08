
idf_component_register(
    SRCS 
        "deca_spi.c"
    INCLUDE_DIRS 
        "include"
    REQUIRES 
        deca_gpio
        decadriver
        esp_driver_gpio 
        esp_driver_spi
        freertos
    
)
