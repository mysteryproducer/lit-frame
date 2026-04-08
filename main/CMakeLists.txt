idf_component_register(SRCS "frame.cpp"
                    PRIV_REQUIRES 
                    esp_driver_ledc esp_driver_gpio esp_driver_i2c
                    espp__timer vl53l8cx
                    INCLUDE_DIRS ".")
# main esptool_py timer logger vl53l i2c