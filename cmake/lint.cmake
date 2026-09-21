###############################################################################
## Format and lint targets (clang-format / clang-tidy from the firmware configs)
###############################################################################

find_program(CLANG_FORMAT clang-format)
find_program(CLANG_TIDY clang-tidy)

function(micras_add_lint_targets)
    set(SOURCES ${ARGN})

    if(NOT CLANG_FORMAT OR NOT CLANG_TIDY)
        message(WARNING "clang-format/clang-tidy not found; the format and lint targets are unavailable")
        return()
    endif()

    add_custom_target(format
        COMMAND "${CLANG_FORMAT}" -style=file -i ${SOURCES}
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        COMMENT "Formatting harness sources"
        VERBATIM USES_TERMINAL
    )

    add_custom_target(format-check
        COMMAND "${CLANG_FORMAT}" -style=file --dry-run --Werror ${SOURCES}
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        COMMENT "Checking harness formatting"
        VERBATIM USES_TERMINAL
    )

    list(FILTER SOURCES INCLUDE REGEX "\\.cpp$")

    # Anchored so the pinned firmware submodule headers are never linted.
    add_custom_target(lint
        COMMAND "${CLANG_TIDY}" --quiet --warnings-as-errors=*
                "--header-filter=^${CMAKE_SOURCE_DIR}/(include|config|tests)/"
                -p "${CMAKE_BINARY_DIR}" ${SOURCES}
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
        COMMENT "Linting harness sources"
        VERBATIM USES_TERMINAL
    )
endfunction()
