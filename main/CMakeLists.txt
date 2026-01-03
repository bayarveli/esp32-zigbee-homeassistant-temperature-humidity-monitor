idf_component_register(SRCS "sensor_dht22.cpp" "main.cpp"
                       PRIV_REQUIRES spi_flash nvs_flash dht
                       INCLUDE_DIRS "")
