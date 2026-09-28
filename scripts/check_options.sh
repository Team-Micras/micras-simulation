#!/bin/bash
# check_options.sh <source dir> <build dir> <generator> <C compiler> <C++ compiler> <build type> <werror> <cmake argument>...
#
# Configures and builds the simulator with every optional part off, the viewer, the video, the bridge,
# the tests and the targets, so that their stubs keep compiling. The remaining arguments go to CMake,
# for example the dependencies' sources the parent build already fetched.

set -euo pipefail

if [[ $# -lt 7 ]]; then
    echo "usage: $0 <source dir> <build dir> <generator> <C compiler> <C++ compiler> <build type> <werror> <cmake argument>..." >&2
    exit 2
fi

source_dir=$1
build_dir=$2
generator=$3
c_compiler=$4
cxx_compiler=$5
build_type=$6
werror=$7
shift 7

cmake -S "${source_dir}" -B "${build_dir}" -G "${generator}" \
    -DCMAKE_BUILD_TYPE="${build_type}" \
    -DCMAKE_C_COMPILER="${c_compiler}" \
    -DCMAKE_CXX_COMPILER="${cxx_compiler}" \
    -DMICRAS_SIM_VIEWER=OFF \
    -DMICRAS_SIM_VIDEO=OFF \
    -DMICRAS_SIM_BRIDGE=OFF \
    -DMICRAS_SIM_TESTS=OFF \
    -DMICRAS_SIM_TARGETS=OFF \
    -DMICRAS_SIM_WERROR="${werror}" \
    "$@" > "${build_dir}.log"

cmake --build "${build_dir}" >> "${build_dir}.log"
echo "ok   every option off configures and builds (${build_dir})"
