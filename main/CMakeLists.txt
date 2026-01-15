idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES spi_flash nvs_flash esp_driver_i2c bme280
                       INCLUDE_DIRS "")
