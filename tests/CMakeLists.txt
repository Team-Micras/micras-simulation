###############################################################################
## Engine unit tests (GoogleTest + CTest), on a tiny robot of their own
###############################################################################

include(GoogleTest)

add_executable(micras_sim_tests
    unit/cli_test.cpp
    unit/clock_test.cpp
    unit/csv_recorder_test.cpp
    unit/csv_writer_test.cpp
    unit/devices_test.cpp
    unit/event_log_test.cpp
    unit/firmware_thread_test.cpp
    unit/maze_test.cpp
    unit/noise_test.cpp
    unit/robot_description_test.cpp
    unit/run_metadata_test.cpp
    unit/scenario_test.cpp
    unit/serial_bus_test.cpp
    unit/simulation_test.cpp
)

# The bridge tests need the real server, which -DMICRAS_SIM_BRIDGE=OFF replaces by a stub.
if(MICRAS_SIM_BRIDGE)
    target_sources(micras_sim_tests PRIVATE unit/web_socket_server_test.cpp)
endif()

target_link_libraries(micras_sim_tests PRIVATE micras::sim_app GTest::gtest_main)
target_compile_options(micras_sim_tests PRIVATE -Wall -Wextra)
target_compile_definitions(micras_sim_tests PRIVATE
    MICRAS_TEST_MODEL="${CMAKE_CURRENT_SOURCE_DIR}/models/tiny_robot.xml"
    MICRAS_TEST_ROBOT="${CMAKE_CURRENT_SOURCE_DIR}/models/tiny_robot.toml"
    MICRAS_TEST_MAZES_DIR="${PROJECT_SOURCE_DIR}/engine/arenas/maze/mazes"
)

gtest_discover_tests(micras_sim_tests)
