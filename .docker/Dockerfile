# syntax=docker/dockerfile:1.27

# >>> host stage >>>
###############################################################################
# Host tools: shared byte for byte by micras-lib, micras-simulation and       #
# MicrasFirmware, whose CI compares the lines between the two markers         #
###############################################################################
FROM ubuntu:26.04 AS host

# The compilers and the clang tooling are pinned to a major version, so that a distribution update
# can move neither the generated code nor the formatting rules and the enabled check set
ARG HOST_GCC_VERSION=15
ARG HOST_CLANG_VERSION=22

# The apt caches are kept in BuildKit cache mounts instead of the image layers, so the hook that
# deletes the downloaded packages after every install has to go
RUN rm -f /etc/apt/apt.conf.d/docker-clean

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ca-certificates \
        cmake \
        doxygen \
        git \
        graphviz \
        ninja-build \
        python3 \
        python3-matplotlib \
        python3-numpy \
        "gcc-${HOST_GCC_VERSION}" \
        "g++-${HOST_GCC_VERSION}" \
        "clang-${HOST_CLANG_VERSION}" \
        "clang-format-${HOST_CLANG_VERSION}" \
        "clang-tidy-${HOST_CLANG_VERSION}"

WORKDIR /project
# <<< host stage <<<

###############################################################################
# The simulator: what the viewer, the video and their checks need on top     #
###############################################################################
FROM host AS simulator

ARG HOST_CLANG_VERSION=22

# GLFW and the OpenGL and EGL development files for the viewer and the video, Mesa's software renderer
# for a machine with no GPU, Xvfb for a window with no display, and ffmpeg, which encodes the video.
# clang's own sanitizer headers let clang-tidy read a sanitized build: MuJoCo's headers include them.
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        "libclang-rt-${HOST_CLANG_VERSION}-dev" \
        ffmpeg \
        libegl-dev \
        libegl-mesa0 \
        libgl-dev \
        libgl1-mesa-dri \
        libglfw3-dev \
        libglx-mesa0 \
        libopengl-dev \
        xauth \
        xvfb
