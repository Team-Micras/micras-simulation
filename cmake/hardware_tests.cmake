###############################################################################
## One executable per firmware hardware test
###############################################################################

# The tests are compiled unchanged: their main is renamed so the harness can
# supply its own and run theirs inside the firmware thread.
function(micras_add_hardware_test NAME SOURCE)
    set(TARGET "micras_${NAME}")

    add_executable(${TARGET}
        "${MICRAS_FIRMWARE_DIR}/${SOURCE}"
        "${CMAKE_SOURCE_DIR}/src/sim/app/hardware_test_main.cpp"
    )

    set_source_files_properties("${MICRAS_FIRMWARE_DIR}/${SOURCE}"
        PROPERTIES COMPILE_DEFINITIONS "main=micras_test_main"
    )

    target_include_directories(${TARGET} PRIVATE "${CMAKE_SOURCE_DIR}/tests/include")
    target_link_libraries(${TARGET} PRIVATE micras_sim_app)

    set_target_properties(${TARGET} PROPERTIES
        OUTPUT_NAME ${NAME}
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/hardware_tests"
        BUILD_RPATH "${MUJOCO_DIR}/lib"
        INSTALL_RPATH "${MUJOCO_DIR}/lib"
    )
endfunction()

function(micras_add_hardware_tests)
    foreach(SOURCE ${ARGN})
        get_filename_component(NAME "${SOURCE}" NAME_WLE)
        micras_add_hardware_test("${NAME}" "${SOURCE}")
        list(APPEND TARGETS "micras_${NAME}")
    endforeach()

    add_custom_target(hardware_tests DEPENDS ${TARGETS})
endfunction()
