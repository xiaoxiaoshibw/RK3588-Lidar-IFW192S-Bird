idf_component_register(SRCS "main.cpp" "replay_wire.cpp" "tracker.cpp" "geometry.cpp"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_uart esp_driver_usb_serial_jtag esp_timer esp_psram)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
