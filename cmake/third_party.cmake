###############################################################################
## Third-party dependencies, pinned and fetched at configure time
###############################################################################

# Each is pinned by commit and fetched as the archive of that commit, checked against its SHA-256:
# a commit cannot be cloned shallow, and a full clone of ImGui alone is 132 MB of history.

include(FetchContent)

FetchContent_Declare(picosha2
    URL      "https://github.com/okdshin/PicoSHA2/archive/27fcf6979298949e8a462e16d09a0351c18fcaf2.tar.gz"
    URL_HASH "SHA256=18d82bb79c021ccf4ce58125b64691accef54237ba5194462740bacf8b39d8a9"
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SYSTEM
)
FetchContent_MakeAvailable(picosha2)

# Robot descriptions and scenarios are TOML. v3.4.0.
FetchContent_Declare(tomlplusplus
    URL      "https://github.com/marzer/tomlplusplus/archive/30172438cee64926dc41fdd9c11fb3ba5b2ba9de.tar.gz"
    URL_HASH "SHA256=291254ffe7f2433f90deef878d0d9335534a350a958ea23ecf511b7b2277bf7f"
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SYSTEM
)
FetchContent_MakeAvailable(tomlplusplus)

if(MICRAS_SIM_VIEWER)
    # Neither ships a CMakeLists, so both are compiled here from their sources. ImGui v1.91.5, ImPlot v0.16.
    FetchContent_Declare(imgui
        URL      "https://github.com/ocornut/imgui/archive/f401021d5a5d56fe2304056c391e78f81c8d4b8f.tar.gz"
        URL_HASH "SHA256=0a363f5c9dd263233a23087c7f68b3c41a050c9c9b4cf5cf02bf843e672b3464"
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        SYSTEM
    )
    FetchContent_MakeAvailable(imgui)

    FetchContent_Declare(implot
        URL      "https://github.com/epezent/implot/archive/18c72431f8265e2b0b5378a3a73d8a883b2175ff.tar.gz"
        URL_HASH "SHA256=4787c77e6050f3bdc19f39eecf87d5b321bd3096321142b63f8169e1aa8f9b34"
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        SYSTEM
    )
    FetchContent_MakeAvailable(implot)

    add_library(micras_sim_imgui STATIC
        "${imgui_SOURCE_DIR}/imgui.cpp"
        "${imgui_SOURCE_DIR}/imgui_draw.cpp"
        "${imgui_SOURCE_DIR}/imgui_tables.cpp"
        "${imgui_SOURCE_DIR}/imgui_widgets.cpp"
        "${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp"
        "${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp"
        "${implot_SOURCE_DIR}/implot.cpp"
        "${implot_SOURCE_DIR}/implot_items.cpp"
    )

    target_include_directories(micras_sim_imgui SYSTEM PUBLIC
        "${imgui_SOURCE_DIR}"
        "${imgui_SOURCE_DIR}/backends"
        "${implot_SOURCE_DIR}"
    )

    find_package(glfw3 QUIET)

    if(glfw3_FOUND)
        target_link_libraries(micras_sim_imgui PUBLIC glfw)
    else()
        find_library(IMGUI_GLFW_LIBRARY NAMES glfw glfw3 REQUIRED)
        target_link_libraries(micras_sim_imgui PUBLIC "${IMGUI_GLFW_LIBRARY}")
    endif()
endif()

if(MICRAS_SIM_BRIDGE)
    # Normal variables, which its option() calls honor (CMP0077), so nothing lands in a parent's cache. v11.4.5.
    set(USE_ZLIB OFF)
    set(USE_TLS OFF)
    set(IXWEBSOCKET_INSTALL OFF)

    FetchContent_Declare(ixwebsocket
        URL      "https://github.com/machinezone/IXWebSocket/archive/c5a02f1066fb0fde48f80f51178429a27f689a39.tar.gz"
        URL_HASH "SHA256=ef272693e67daef33275daa8d3685f48e8fe4dbe098338750f9dad3013016d96"
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        SYSTEM
    )
    FetchContent_MakeAvailable(ixwebsocket)
endif()

if(MICRAS_SIM_TESTS)
    # doctest v2.5.3, the commit micras-lib's tests use. Its headers are system headers when it is added
    # to another project, so its macros' own code raises no warning here.
    set(DOCTEST_NO_INSTALL ON)

    FetchContent_Declare(doctest
        URL      "https://github.com/doctest/doctest/archive/2d0a9359a60c51affe2a9bebb1be1dca47868151.tar.gz"
        URL_HASH "SHA256=e64542c4ea68e9f381ccf6eae924cfdd652567c87c142d76fe92644fb4608149"
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
        SYSTEM
    )
    FetchContent_MakeAvailable(doctest)

    include("${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake")
endif()
