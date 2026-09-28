###############################################################################
## Third-party dependencies, pinned and fetched at configure time
###############################################################################

include(FetchContent)

FetchContent_Declare(picosha2
    GIT_REPOSITORY https://github.com/okdshin/PicoSHA2.git
    GIT_TAG        27fcf6979298949e8a462e16d09a0351c18fcaf2
)
FetchContent_MakeAvailable(picosha2)

# Robot descriptions and scenarios are TOML.
FetchContent_Declare(tomlplusplus
    GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
    GIT_TAG        v3.4.0
)
FetchContent_MakeAvailable(tomlplusplus)

# ST's register driver of the LSM6DSV, the tag the firmware fetches. The IMU proxy's
# configuration names the driver's enums, and a robot target that compiles the proxy
# compiles the driver's source too.
FetchContent_Declare(lsm6dsv
    GIT_REPOSITORY https://github.com/STMicroelectronics/lsm6dsv-pid.git
    GIT_TAG        v5.1.1
)
FetchContent_MakeAvailable(lsm6dsv)

add_library(lsm6dsv_headers INTERFACE)
target_include_directories(lsm6dsv_headers SYSTEM INTERFACE "${lsm6dsv_SOURCE_DIR}")

if(MICRAS_VIEWER)
    # Neither ships a CMakeLists, so both are compiled here from their sources.
    FetchContent_Declare(imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG        v1.91.5
    )
    FetchContent_MakeAvailable(imgui)

    FetchContent_Declare(implot
        GIT_REPOSITORY https://github.com/epezent/implot.git
        GIT_TAG        v0.16
    )
    FetchContent_MakeAvailable(implot)

    add_library(imgui STATIC
        "${imgui_SOURCE_DIR}/imgui.cpp"
        "${imgui_SOURCE_DIR}/imgui_draw.cpp"
        "${imgui_SOURCE_DIR}/imgui_tables.cpp"
        "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
        "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
        "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp"
        "${implot_SOURCE_DIR}/implot.cpp"
        "${implot_SOURCE_DIR}/implot_items.cpp"
    )

    target_include_directories(imgui SYSTEM PUBLIC
        "${imgui_SOURCE_DIR}"
        "${imgui_SOURCE_DIR}/backends"
        "${implot_SOURCE_DIR}"
    )

    find_package(glfw3 QUIET)

    if(glfw3_FOUND)
        target_link_libraries(imgui PUBLIC glfw)
    else()
        find_library(IMGUI_GLFW_LIBRARY NAMES glfw glfw3 REQUIRED)
        target_link_libraries(imgui PUBLIC "${IMGUI_GLFW_LIBRARY}")
    endif()
endif()

if(MICRAS_BRIDGE)
    set(USE_ZLIB OFF CACHE BOOL "" FORCE)
    set(USE_TLS OFF CACHE BOOL "" FORCE)
    set(IXWEBSOCKET_INSTALL OFF CACHE BOOL "" FORCE)

    FetchContent_Declare(ixwebsocket
        GIT_REPOSITORY https://github.com/machinezone/IXWebSocket.git
        GIT_TAG        v11.4.5
    )
    FetchContent_MakeAvailable(ixwebsocket)
endif()

if(MICRAS_TESTS)
    FetchContent_Declare(googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG        b514bdc898e2951020cbdca1304b75f5950d1f59
    )
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()
