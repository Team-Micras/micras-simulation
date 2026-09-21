###############################################################################
## Unit tests (GoogleTest + CTest)
###############################################################################

include(GoogleTest)

add_executable(micras_sim_tests
    unit/cli_test.cpp
    unit/clock_test.cpp
    unit/firmware_thread_test.cpp
    unit/simulation_test.cpp
    unit/scenario_test.cpp
    unit/serial_bus_test.cpp
    unit/packet_framer_test.cpp
    unit/telemetry_test.cpp
    unit/csv_writer_test.cpp
    unit/csv_recorder_test.cpp
    unit/run_metadata_test.cpp
    unit/proxy_boundary_test.cpp
    unit/shadow_conformance_test.cpp
)

target_link_libraries(micras_sim_tests PRIVATE micras_sim_app GTest::gtest_main)
target_compile_options(micras_sim_tests PRIVATE -Wall -Wextra)

# The conformance test initialises only the fields it shares with the firmware.
set_source_files_properties(unit/shadow_conformance_test.cpp
    PROPERTIES COMPILE_OPTIONS -Wno-missing-field-initializers
)

target_compile_definitions(micras_sim_tests PRIVATE
    MICRAS_TEST_MODEL="${CMAKE_SOURCE_DIR}/models/robot_v2.xml"
)

gtest_discover_tests(micras_sim_tests)
