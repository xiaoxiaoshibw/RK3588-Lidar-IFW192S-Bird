idf_component_register(SRCS "grid_algorithm.cpp" "tracker.cpp" "geometry.cpp"
                       INCLUDE_DIRS ".")
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
