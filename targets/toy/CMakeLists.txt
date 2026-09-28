###############################################################################
## Toy: the simulator's reference target, the smallest robot that uses every feature
###############################################################################

add_library(micras_sim_toy STATIC src/toy_target.cpp)
target_include_directories(micras_sim_toy PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries(micras_sim_toy PUBLIC micras::sim_app)
target_compile_definitions(micras_sim_toy PRIVATE MICRAS_SIM_TOY_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(toy_sim src/main.cpp)
target_link_libraries(toy_sim PRIVATE micras_sim_toy)

micras_apply_warnings(micras_sim_toy toy_sim WERROR ${MICRAS_SIM_WERROR})

###############################################################################
## Checks: a run is reproducible whatever watches it, and matches its baseline
###############################################################################

if(PROJECT_IS_TOP_LEVEL)
    set(MICRAS_SIM_TOY_BASELINE "v1" CACHE STRING "The toy's baseline version, in targets/toy/baselines/")
    set(MICRAS_SIM_TOY_RUNS "${CMAKE_CURRENT_BINARY_DIR}/runs")
    set(MICRAS_SIM_TOY_CHECKED idle drive)

    add_custom_target(micras_sim_toy_check
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${MICRAS_SIM_TOY_RUNS}"
        COMMAND "${PROJECT_SOURCE_DIR}/scripts/check_invariance.sh"
                "$<TARGET_FILE:toy_sim>" "${CMAKE_CURRENT_SOURCE_DIR}/scenarios/drive.toml" "${MICRAS_SIM_TOOLS_DIR}"
                "${MICRAS_SIM_TOY_RUNS}/invariance"
        COMMAND "${PROJECT_SOURCE_DIR}/scripts/check_baseline.sh" compare
                "$<TARGET_FILE:toy_sim>" "${CMAKE_CURRENT_SOURCE_DIR}/scenarios"
                "${CMAKE_CURRENT_SOURCE_DIR}/baselines/${MICRAS_SIM_TOY_BASELINE}" "${MICRAS_SIM_TOOLS_DIR}"
                "${MICRAS_SIM_TOY_RUNS}" "${MICRAS_SIM_EXACT}" ${MICRAS_SIM_TOY_CHECKED}
        COMMENT "Checking the toy target: invariance and baseline"
        VERBATIM USES_TERMINAL
    )
    add_dependencies(micras_sim_toy_check toy_sim)

    add_custom_target(micras_sim_toy_record_baseline
        COMMAND "${CMAKE_COMMAND}" -E make_directory "${MICRAS_SIM_TOY_RUNS}"
        COMMAND "${PROJECT_SOURCE_DIR}/scripts/check_baseline.sh" record
                "$<TARGET_FILE:toy_sim>" "${CMAKE_CURRENT_SOURCE_DIR}/scenarios"
                "${CMAKE_CURRENT_SOURCE_DIR}/baselines/${MICRAS_SIM_TOY_BASELINE}" "${MICRAS_SIM_TOOLS_DIR}"
                "${MICRAS_SIM_TOY_RUNS}" OFF ${MICRAS_SIM_TOY_CHECKED}
        COMMENT "Recording the toy's baseline ${MICRAS_SIM_TOY_BASELINE}"
        VERBATIM USES_TERMINAL
    )
    add_dependencies(micras_sim_toy_record_baseline toy_sim)
endif()
