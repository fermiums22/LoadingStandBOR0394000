cmake_minimum_required(VERSION 3.20)

project(DriveScope LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
)
FetchContent_MakeAvailable(glfw)

FetchContent_Declare(
    imgui_src
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    # Docking branch — required for ImGuiConfigFlags_DockingEnable +
    # ImGuiConfigFlags_ViewportsEnable so tabs can be dragged out of the
    # main window into separate native OS windows.
    GIT_TAG v1.90.9-docking
)
FetchContent_MakeAvailable(imgui_src)

FetchContent_Declare(
    implot_src
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG v0.16
)
FetchContent_MakeAvailable(implot_src)

add_library(imgui STATIC
    ${imgui_src_SOURCE_DIR}/imgui.cpp
    ${imgui_src_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_src_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_src_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_src_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_src_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
    ${imgui_src_SOURCE_DIR}
    ${imgui_src_SOURCE_DIR}/backends
)
target_link_libraries(imgui PUBLIC glfw)

add_library(implot STATIC
    ${implot_src_SOURCE_DIR}/implot.cpp
    ${implot_src_SOURCE_DIR}/implot_items.cpp
)
target_include_directories(implot PUBLIC
    ${implot_src_SOURCE_DIR}
    ${imgui_src_SOURCE_DIR}
)
target_link_libraries(implot PUBLIC imgui)

find_package(OpenGL REQUIRED)

# Исходники задаются звёздочкой по папкам, а не пофайловым списком: новый .cpp
# в src/ или в любой папке src/<подкаталог>/ подхватывается сам, руками сюда
# ничего дописывать не надо. CONFIGURE_DEPENDS заставляет CMake пере-сканировать
# на сборке. src/*.cpp — верхний уровень (main.cpp), src/*/*.cpp — все подпапки
# (can, cli, protocol, ui, transport, ...).
file(GLOB DRIVESCOPE_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/*/*.cpp")

add_executable(drivescope ${DRIVESCOPE_SOURCES})

target_include_directories(drivescope PRIVATE src)
target_link_libraries(drivescope PRIVATE imgui implot glfw OpenGL::GL)

if(WIN32)
    target_compile_definitions(drivescope PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    # wlanapi: enumerate Kitchen_Machine_* SSIDs + WlanConnect for AP-mode auto-join.
    target_link_libraries(drivescope PRIVATE ws2_32 comdlg32 iphlpapi wlanapi)
    set_target_properties(drivescope PROPERTIES WIN32_EXECUTABLE TRUE)
endif()

# Static linking on MinGW so the exe is portable without msys64 DLLs.
if(MINGW)
    target_link_options(drivescope PRIVATE
        -static-libgcc
        -static-libstdc++
        "LINKER:-Bstatic,--whole-archive,-lwinpthread,--no-whole-archive,-Bdynamic"
    )
endif()

# After build copy the exe + sample configs into ${PROJECT_SOURCE_DIR}/dist/
# so users have a clear-cut portable folder with one DriveScope.exe to launch.
set(DRIVESCOPE_DIST_DIR "${CMAKE_SOURCE_DIR}/dist" CACHE PATH "DriveScope portable bundle output")
add_custom_command(TARGET drivescope POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${DRIVESCOPE_DIST_DIR}/config"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${DRIVESCOPE_DIST_DIR}/symbols"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${DRIVESCOPE_DIST_DIR}/ssh"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:drivescope>" "${DRIVESCOPE_DIST_DIR}/DriveScope.exe"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/config/drivescope_scenario.json"
        "${DRIVESCOPE_DIST_DIR}/drivescope_scenario.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/config/drivescope_wifi_debug.json"
        "${DRIVESCOPE_DIST_DIR}/config/drivescope_wifi_debug.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/config/drivescope_slcan_debug.json"
        "${DRIVESCOPE_DIST_DIR}/config/drivescope_slcan_debug.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/config/drivescope_tcp_protocol_probe.json"
        "${DRIVESCOPE_DIST_DIR}/config/drivescope_tcp_protocol_probe.json"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/config/drivescope_slcan_protocol_probe.json"
        "${DRIVESCOPE_DIST_DIR}/config/drivescope_slcan_protocol_probe.json"
    COMMAND ${CMAKE_COMMAND}
        "-DSRC=C:/Program Files/PuTTY/plink.exe"
        "-DDST=${DRIVESCOPE_DIST_DIR}/ssh/plink.exe"
        -P "${CMAKE_SOURCE_DIR}/cmake/copy_if_exists.cmake"
    COMMAND ${CMAKE_COMMAND}
        "-DSRC=C:/Program Files/PuTTY/pscp.exe"
        "-DDST=${DRIVESCOPE_DIST_DIR}/ssh/pscp.exe"
        -P "${CMAKE_SOURCE_DIR}/cmake/copy_if_exists.cmake"
    # Bitvise sftpc.exe + sexec.exe: empirically faster and dramatically
    # more reliable on the recovery AP than PuTTY's pscp/plink. Both accept
    # `-hostKeyFp=any` so we don't have to hard-code a per-firmware host
    # key fingerprint in the source. Bundled into dist/ssh/ if installed
    # locally so the production release ships with them (the runtime
    # finders look at ssh/sexec.exe and ssh/sftpc.exe under exeDir first).
    COMMAND ${CMAKE_COMMAND}
        "-DSRC=C:/Program Files (x86)/Bitvise SSH Client/sftpc.exe"
        "-DDST=${DRIVESCOPE_DIST_DIR}/ssh/sftpc.exe"
        -P "${CMAKE_SOURCE_DIR}/cmake/copy_if_exists.cmake"
    COMMAND ${CMAKE_COMMAND}
        "-DSRC=C:/Program Files/Bitvise SSH Client/sftpc.exe"
        "-DDST=${DRIVESCOPE_DIST_DIR}/ssh/sftpc.exe"
        -P "${CMAKE_SOURCE_DIR}/cmake/copy_if_exists.cmake"
    COMMAND ${CMAKE_COMMAND}
        "-DSRC=C:/Program Files (x86)/Bitvise SSH Client/sexec.exe"
        "-DDST=${DRIVESCOPE_DIST_DIR}/ssh/sexec.exe"
        -P "${CMAKE_SOURCE_DIR}/cmake/copy_if_exists.cmake"
    COMMAND ${CMAKE_COMMAND}
        "-DSRC=C:/Program Files/Bitvise SSH Client/sexec.exe"
        "-DDST=${DRIVESCOPE_DIST_DIR}/ssh/sexec.exe"
        -P "${CMAKE_SOURCE_DIR}/cmake/copy_if_exists.cmake"
    # Step 1: refresh dist/symbols/* from current ELFs FIRST. mtime-aware,
    # downgrades elf_export failures to WARNINGs so they can't break the
    # pc-tool build. Release-grade hard-fail variant is in release-all.sh.
    # Doing this BEFORE the dist->build copy below means the build/ tree
    # picks up freshly-regenerated symbols on the very next pc-tool build,
    # not one build behind (which used to be the case when refresh ran last).
    COMMAND ${CMAKE_COMMAND} "-DUMB_ROOT=${CMAKE_SOURCE_DIR}/../.."
        -P "${CMAKE_SOURCE_DIR}/cmake/refresh_symbols.cmake"
    # Step 2: dist/symbols/* -> build/symbols/<name>.json so a drivescope.exe
    # run against the build-tree scenario sees the freshest variable
    # addresses AND the layout mirrors the release zip exactly:
    #
    #   <exe-dir>/DriveScope.exe + <exe-dir>/symbols/symbols_*.json.
    #
    # The default scenario references `symbols/symbols_*.json` (relative),
    # so the same scenario JSON works whether it's loaded from a dev
    # build/pc-tool/ or from a release DriveScope/ unzipped to USB.
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/symbols"
    COMMAND ${CMAKE_COMMAND}
        "-DSRC=${DRIVESCOPE_DIST_DIR}/symbols/symbols_motor.json"
        "-DDST=${CMAKE_BINARY_DIR}/symbols/symbols_motor.json"
        -P "${CMAKE_SOURCE_DIR}/cmake/copy_if_exists.cmake"
    COMMAND ${CMAKE_COMMAND}
        "-DSRC=${DRIVESCOPE_DIST_DIR}/symbols/symbols_rk.json"
        "-DDST=${CMAKE_BINARY_DIR}/symbols/symbols_rk.json"
        -P "${CMAKE_SOURCE_DIR}/cmake/copy_if_exists.cmake"
    COMMAND ${CMAKE_COMMAND}
        "-DSRC=${DRIVESCOPE_DIST_DIR}/symbols/symbols_dd.json"
        "-DDST=${CMAKE_BINARY_DIR}/symbols/symbols_dd.json"
        -P "${CMAKE_SOURCE_DIR}/cmake/copy_if_exists.cmake"
    COMMAND ${CMAKE_COMMAND}
        "-DSRC=${DRIVESCOPE_DIST_DIR}/symbols/symbols_esp32.json"
        "-DDST=${CMAKE_BINARY_DIR}/symbols/symbols_esp32.json"
        -P "${CMAKE_SOURCE_DIR}/cmake/copy_if_exists.cmake"
    COMMENT "Copying DriveScope.exe + configs + symbols to ${DRIVESCOPE_DIST_DIR}/"
    VERBATIM)
