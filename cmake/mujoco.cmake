###############################################################################
## MuJoCo: the pinned release, downloaded unless MUJOCO_DIR names an install
###############################################################################

set(MUJOCO_VERSION "3.14.0")
set(MUJOCO_SHA256 "326f0da78a7767cc18fab7205c993a771b5f19eda913648f958ee3f625240944")

set(MUJOCO_DIR "$ENV{MUJOCO_DIR}" CACHE PATH "Root of a MuJoCo installation; empty downloads the pinned release")

if(MUJOCO_DIR STREQUAL "")
    include(FetchContent)

    if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux" OR NOT CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64")
        message(FATAL_ERROR "no pinned MuJoCo release for this platform; set MUJOCO_DIR to an installation")
    endif()

    FetchContent_Declare(mujoco
        URL "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz"
        URL_HASH "SHA256=${MUJOCO_SHA256}"
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_MakeAvailable(mujoco)
    set(MUJOCO_ROOT "${mujoco_SOURCE_DIR}")
else()
    set(MUJOCO_ROOT "${MUJOCO_DIR}")
endif()

# Found afresh on every configure, so a new pin or MUJOCO_DIR never keeps the old library.
unset(MUJOCO_HEADER CACHE)
unset(MUJOCO_LIBRARY CACHE)
find_file(MUJOCO_HEADER mujoco/mujoco.h PATHS "${MUJOCO_ROOT}/include" NO_DEFAULT_PATH NO_CACHE REQUIRED)
find_library(MUJOCO_LIBRARY mujoco PATHS "${MUJOCO_ROOT}/lib" NO_DEFAULT_PATH NO_CACHE REQUIRED)

# The release's include directory also carries the ImGui and ImPlot of its own tools,
# newer than the ones the viewer builds with, so only its mujoco/ headers are exposed.
set(MUJOCO_INCLUDE "${PROJECT_BINARY_DIR}/mujoco_include")
file(MAKE_DIRECTORY "${MUJOCO_INCLUDE}")
file(CREATE_LINK "${MUJOCO_ROOT}/include/mujoco" "${MUJOCO_INCLUDE}/mujoco" SYMBOLIC)

add_library(mujoco::mujoco SHARED IMPORTED)
set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE}"
)
