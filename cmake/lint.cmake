###############################################################################
## Format and lint targets (clang-format / clang-tidy from the firmware configs)
###############################################################################

# Pinned to one major version, like the firmware's container: clang-format 19 and later
# lay designated initializers out differently, so a newer one would reformat every file.
find_program(CLANG_FORMAT NAMES clang-format-18 clang-format)
find_program(CLANG_TIDY NAMES clang-tidy-18 clang-tidy)

function(micras_add_lint_targets)
    set(SOURCES ${ARGN})

    if(NOT CLANG_FORMAT OR NOT CLANG_TIDY)
        message(WARNING "clang-format/clang-tidy not found; the format and lint targets are unavailable")
        return()
    endif()

    add_custom_target(micras_sim_format
        COMMAND "${CLANG_FORMAT}" -style=file -i ${SOURCES}
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        COMMENT "Formatting harness sources"
        VERBATIM USES_TERMINAL
    )

    add_custom_target(micras_sim_format_check
        COMMAND "${CLANG_FORMAT}" -style=file --dry-run --Werror ${SOURCES}
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        COMMENT "Checking harness formatting"
        VERBATIM USES_TERMINAL
    )

    list(FILTER SOURCES INCLUDE REGEX "\\.cpp$")

    # Anchored so the pinned firmware submodule headers are never linted.
    # The firmware computes the shape of every turn in a constant expression, which takes more
    # evaluation steps than clang allows by default; the step limit is the firmware's own lint's.
    add_custom_target(micras_sim_lint
        COMMAND "${CLANG_TIDY}" --quiet --warnings-as-errors=*
                "--header-filter=^${PROJECT_SOURCE_DIR}/(engine|view|bridge|app|tests|targets/[a-z_]+/(include|config|tests))/"
                --extra-arg=-fconstexpr-steps=1000000000
                -p "${PROJECT_BINARY_DIR}" ${SOURCES}
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        COMMENT "Linting harness sources"
        VERBATIM USES_TERMINAL
    )
endfunction()
