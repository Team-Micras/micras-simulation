###############################################################################
## Project Warning Flags
###############################################################################

# The canonical copy lives in micras-lib; micras-simulation and MicrasFirmware carry the same file
# byte for byte, and the firmware's CI compares them. The -Werror switch belongs to each repository
# (MICRAS_LIB_WERROR, MICRAS_SIM_WERROR, MICRAS_WERROR) and is passed to micras_apply_warnings.

# The -Wduplicated-* and -Wuseless-cast flags are GCC only. -fno-common is a code generation flag:
# GCC's default for C and a no-op for C++, kept so that the robot's image stays the same.
set(MICRAS_WARNING_OPTIONS
    -Wall
    -Wextra
    -Wpedantic
    -Wcast-align
    -Wcast-qual
    -Wdouble-promotion
    $<$<CXX_COMPILER_ID:GNU>:-Wduplicated-branches>
    $<$<CXX_COMPILER_ID:GNU>:-Wduplicated-cond>
    -Wfloat-conversion
    -Wformat=2
    -Wimplicit-fallthrough
    -Wmisleading-indentation
    -Wnull-dereference
    -Wsign-conversion
    -Wundef
    -Wunused-parameter
    -fno-common

    # C++ only, they make the C compiler warn about the command line itself
    $<$<COMPILE_LANGUAGE:CXX>:-Wnon-virtual-dtor>
    $<$<COMPILE_LANGUAGE:CXX>:-Wold-style-cast>
    $<$<COMPILE_LANGUAGE:CXX>:-Woverloaded-virtual>
    $<$<AND:$<COMPILE_LANGUAGE:CXX>,$<CXX_COMPILER_ID:GNU>>:-Wuseless-cast>
)

# Kept globally as well, so that the function works in any directory, whichever of the three
# repositories included this file first
set_property(GLOBAL PROPERTY MICRAS_WARNING_OPTIONS ${MICRAS_WARNING_OPTIONS})

# micras_apply_warnings(<target>... [WERROR <bool>])
#
# Applied per target, so that the generated sources are never compiled with them, and as options
# rather than through a target, which a static library would carry in its interface. The targets
# are real names, not aliases.
function(micras_apply_warnings)
    cmake_parse_arguments(PARSE_ARGV 0 MICRAS_WARNINGS "" "WERROR" "")
    get_property(WARNING_OPTIONS GLOBAL PROPERTY MICRAS_WARNING_OPTIONS)

    if(MICRAS_WARNINGS_WERROR)
        list(APPEND WARNING_OPTIONS -Werror)
    endif()

    foreach(TARGET_NAME ${MICRAS_WARNINGS_UNPARSED_ARGUMENTS})
        target_compile_options(${TARGET_NAME} PRIVATE ${WARNING_OPTIONS})
    endforeach()
endfunction()
