###############################################################################
## Format and lint targets, with the configuration shared by the three repositories
###############################################################################

# The clang tools are found by their versioned names: another major version formats and lints
# differently, so a missing one is an error instead of a silent fallback to whatever is installed.
set(MICRAS_SIM_CLANG_VERSION 22)

foreach(TOOL clang-format clang-tidy run-clang-tidy clang-apply-replacements)
    string(TOUPPER "MICRAS_${TOOL}" VARIABLE)
    string(REPLACE "-" "_" VARIABLE "${VARIABLE}")
    find_program(${VARIABLE} NAMES "${TOOL}-${MICRAS_SIM_CLANG_VERSION}")

    if(NOT ${VARIABLE})
        message(FATAL_ERROR
            "${TOOL}-${MICRAS_SIM_CLANG_VERSION} was not found. The style targets need clang ${MICRAS_SIM_CLANG_VERSION}'s "
            "clang-format, clang-tidy, run-clang-tidy and clang-apply-replacements by their versioned names.")
    endif()
endforeach()

# micras_sim_add_lint_targets(FORMAT <files>... LINT <sources>...)
#
# micras_sim_format and micras_sim_format_check cover FORMAT, micras_sim_lint and micras_sim_lint_fix run
# clang-tidy over the LINT sources, which must be in the compilation database.
function(micras_sim_add_lint_targets)
    cmake_parse_arguments(PARSE_ARGV 0 MICRAS_SIM_STYLE "" "" "FORMAT;LINT")

    add_custom_target(micras_sim_format
        COMMAND "${MICRAS_CLANG_FORMAT}" -style=file -i ${MICRAS_SIM_STYLE_FORMAT}
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        COMMENT "Formatting the simulator's sources"
        VERBATIM USES_TERMINAL
    )

    add_custom_target(micras_sim_format_check
        COMMAND "${MICRAS_CLANG_FORMAT}" -style=file --dry-run --Werror ${MICRAS_SIM_STYLE_FORMAT}
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        COMMENT "Checking the simulator's formatting"
        VERBATIM USES_TERMINAL
    )

    # The host compiler needs no extra argument, unlike the robot's cross compiler.
    set(MICRAS_TIDY_EXTRA_ARGS "")
    set(LINT_SCRIPT "${PROJECT_BINARY_DIR}/run_clang_tidy.sh")
    configure_file("${PROJECT_SOURCE_DIR}/cmake/templates/run_clang_tidy.sh.in" "${LINT_SCRIPT}" @ONLY)

    add_custom_target(micras_sim_lint
        COMMAND "${LINT_SCRIPT}" ${MICRAS_SIM_STYLE_LINT}
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        COMMENT "Linting the simulator's sources"
        VERBATIM USES_TERMINAL
    )

    add_custom_target(micras_sim_lint_fix
        COMMAND "${LINT_SCRIPT}" --fix ${MICRAS_SIM_STYLE_LINT}
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        COMMENT "Linting the simulator's sources and applying the fixes"
        VERBATIM USES_TERMINAL
    )
endfunction()
