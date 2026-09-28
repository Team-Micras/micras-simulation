###############################################################################
## Chip models: SPI devices for the host micras_hal
###############################################################################

# They know the SPI device slot of the host HAL and nothing else: no Cube layer, no
# firmware, no engine. A robot's bindings attach them to its buses and feed them.
add_library(micras_hal_host_models STATIC
    src/as5047u_model.cpp
    src/lsm6dsv_model.cpp
)
add_library(micras::hal_host_models ALIAS micras_hal_host_models)
target_include_directories(micras_hal_host_models PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
    "${MICRAS_SIM_HAL_HOST_INCLUDE}"
)
target_compile_features(micras_hal_host_models PUBLIC cxx_std_23)
target_compile_options(micras_hal_host_models PRIVATE -Wall -Wextra)

if(MICRAS_TESTS)
    include(GoogleTest)

    # ST's register driver header names the registers the tests address, so a model
    # register at another address than the driver's fails here.
    add_executable(micras_hal_host_models_tests
        tests/as5047u_model_test.cpp
        tests/lsm6dsv_model_test.cpp
    )
    target_link_libraries(micras_hal_host_models_tests PRIVATE
        micras_hal_host_models lsm6dsv_headers GTest::gtest_main
    )
    target_compile_options(micras_hal_host_models_tests PRIVATE -Wall -Wextra)

    gtest_discover_tests(micras_hal_host_models_tests)
endif()
