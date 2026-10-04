###############################################################################
## The simulator's checks, as targets over the scripts of scripts/ (top level only)
###############################################################################

# micras_sim_collect_targets(<variable> <directory>)
#
# Every executable and library the simulator itself defines in <directory> and below, not the fetched
# dependencies', so that a check can depend on the whole build.
function(micras_sim_collect_targets VARIABLE DIRECTORY)
    get_property(TARGETS DIRECTORY "${DIRECTORY}" PROPERTY BUILDSYSTEM_TARGETS)
    get_property(SUBDIRECTORIES DIRECTORY "${DIRECTORY}" PROPERTY SUBDIRECTORIES)
    set(COLLECTED ${${VARIABLE}})

    foreach(TARGET_NAME IN LISTS TARGETS)
        get_target_property(TYPE ${TARGET_NAME} TYPE)

        if(TYPE MATCHES "^(EXECUTABLE|STATIC_LIBRARY|SHARED_LIBRARY)$")
            list(APPEND COLLECTED ${TARGET_NAME})
        endif()
    endforeach()

    foreach(SUBDIRECTORY IN LISTS SUBDIRECTORIES)
        if(SUBDIRECTORY MATCHES "^${PROJECT_BINARY_DIR}/_deps/" OR NOT SUBDIRECTORY MATCHES "^${PROJECT_SOURCE_DIR}/")
            continue()
        endif()

        micras_sim_collect_targets(COLLECTED "${SUBDIRECTORY}")
    endforeach()

    set(${VARIABLE} ${COLLECTED} PARENT_SCOPE)
endfunction()

set(MICRAS_SIM_SCRIPTS_DIR "${PROJECT_SOURCE_DIR}/scripts")

# The options check reuses what this build fetched, so it downloads nothing.
add_custom_target(micras_sim_check_options
    COMMAND "${MICRAS_SIM_SCRIPTS_DIR}/check_options.sh"
            "${PROJECT_SOURCE_DIR}" "${PROJECT_BINARY_DIR}-minimal" "${CMAKE_GENERATOR}"
            "${CMAKE_C_COMPILER}" "${CMAKE_CXX_COMPILER}" "${CMAKE_BUILD_TYPE}" "${MICRAS_SIM_WERROR}"
            "-DMUJOCO_DIR=${MUJOCO_ROOT}"
            "-DFETCHCONTENT_SOURCE_DIR_PICOSHA2=${picosha2_SOURCE_DIR}"
            "-DFETCHCONTENT_SOURCE_DIR_TOMLPLUSPLUS=${tomlplusplus_SOURCE_DIR}"
    COMMENT "Building with every option off"
    VERBATIM USES_TERMINAL
)

add_custom_target(micras_sim_check_generic
    COMMAND "${MICRAS_SIM_SCRIPTS_DIR}/check_generic.sh" "${PROJECT_SOURCE_DIR}"
    COMMENT "Checking that nothing outside targets/ names a robot"
    VERBATIM USES_TERMINAL
)

# The build, every unit test, the options, the generic check and the toy target's checks.
add_custom_target(micras_sim_check
    COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${PROJECT_BINARY_DIR}" --output-on-failure --parallel
    COMMENT "Running the unit tests"
    VERBATIM USES_TERMINAL
)

micras_sim_collect_targets(MICRAS_SIM_BUILD_TARGETS "${PROJECT_SOURCE_DIR}")
add_dependencies(micras_sim_check micras_sim_check_options micras_sim_check_generic ${MICRAS_SIM_BUILD_TARGETS})

if(TARGET micras_sim_toy_check)
    add_dependencies(micras_sim_check micras_sim_toy_check)
endif()
