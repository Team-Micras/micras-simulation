###############################################################################
## MuJoCo (tarball install, no CMake config shipped)
###############################################################################

set(MUJOCO_DIR "$ENV{HOME}/.mujoco/mujoco-3.3.6" CACHE PATH "Root of the MuJoCo installation")

find_file(MUJOCO_HEADER mujoco/mujoco.h PATHS "${MUJOCO_DIR}/include" NO_DEFAULT_PATH REQUIRED)
find_library(MUJOCO_LIBRARY mujoco PATHS "${MUJOCO_DIR}/lib" NO_DEFAULT_PATH REQUIRED)

add_library(mujoco::mujoco SHARED IMPORTED)
set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_DIR}/include"
)
