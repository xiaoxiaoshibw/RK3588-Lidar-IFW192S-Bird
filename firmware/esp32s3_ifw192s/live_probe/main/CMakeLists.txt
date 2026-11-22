set(CORE "../../../esp32s3_bird_core")
idf_component_register(
    SRCS "main.cpp" "${CORE}/tracker.cpp" "${CORE}/geometry.cpp" "${CORE}/grid_algorithm.cpp"
    INCLUDE_DIRS "../.." "../../burst_probe/main"
    PRIV_REQUIRES esp_driver_gpio esp_driver_spi esp_driver_uart esp_timer esp_psram
)
