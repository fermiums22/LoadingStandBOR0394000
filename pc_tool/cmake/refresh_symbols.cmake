# -----------------------------------------------------------------------------
#  refresh_symbols.cmake — invoked as `cmake -DUMB_ROOT=... -P this`.
#
#  Same purpose as tools/refresh-drivescope-symbols.sh: regenerate
#  tools/pc-tool/dist/symbols/symbols_<motor|rk|esp32|dd>.json from the
#  current ELFs.  Pure CMake (no shell dependency) so it works from any
#  invocation context: PowerShell, cmd, MinGW, Git Bash, or VS Code.
#
#  Behaviour:
#    - missing ELF       -> skip with a notice (firmware not built yet).
#    - JSON >= ELF mtime -> skip (already fresh).
#    - elf_export crash  -> WARNING (never FATAL_ERROR); pc-tool build
#                           continues so a borked exporter on one target
#                           can't lock the developer out of building the
#                           DriveScope GUI itself.
# -----------------------------------------------------------------------------

if(NOT DEFINED UMB_ROOT)
    message(FATAL_ERROR "UMB_ROOT must be passed: cmake -DUMB_ROOT=<path> -P refresh_symbols.cmake")
endif()

set(EXPORTER "${UMB_ROOT}/tools/pc-tool/dist/elf_export.exe")
set(OUT_DIR  "${UMB_ROOT}/tools/pc-tool/dist/symbols")

if(NOT EXISTS "${EXPORTER}")
    message(STATUS "refresh_symbols: ${EXPORTER} not built yet — skipping symbol refresh")
    return()
endif()

file(MAKE_DIRECTORY "${OUT_DIR}")

function(to_windows_export_path input output_var)
    set(path "${input}")
    if(path MATCHES "^/mnt/([A-Za-z])/(.*)$")
        string(TOUPPER "${CMAKE_MATCH_1}" drive)
        set(rest "${CMAKE_MATCH_2}")
        string(REPLACE "/" "\\" rest "${rest}")
        set(path "${drive}:\\${rest}")
    elseif(path MATCHES "^/([A-Za-z])/(.*)$")
        string(TOUPPER "${CMAKE_MATCH_1}" drive)
        set(rest "${CMAKE_MATCH_2}")
        string(REPLACE "/" "\\" rest "${rest}")
        set(path "${drive}:\\${rest}")
    endif()
    set(${output_var} "${path}" PARENT_SCOPE)
endfunction()

# tag <-> ELF source path.
#
# Each row is:
#   tag|path_checked_by_cmake|path_passed_to_elf_export
#
# dd lives in WSL. Linux CMake can only test the native /home path, while the
# Windows-native elf_export.exe needs the \\wsl.localhost\... UNC path.
set(TARGETS
    "motor|${UMB_ROOT}/MOTOR/app_stm32f4_motor/Debug/app_stm32f4_motor.elf|${UMB_ROOT}/MOTOR/app_stm32f4_motor/Debug/app_stm32f4_motor.elf"
    "rk|${UMB_ROOT}/RK/app_stm32l4_rk/Debug/app_stm32l4_rk.elf|${UMB_ROOT}/RK/app_stm32l4_rk/Debug/app_stm32l4_rk.elf"
    "esp32|${UMB_ROOT}/mainPCB/app_esp32_dd/.pio/build/esp32dev/firmware.elf|${UMB_ROOT}/mainPCB/app_esp32_dd/.pio/build/esp32dev/firmware.elf"
    "dd|/home/fermiums/linux_ssd202d_dd/sdk/app_ssd202d_ddv2/app/app_dd|\\\\wsl.localhost\\Ubuntu_DDV2\\home\\fermiums\\linux_ssd202d_dd\\sdk\\app_ssd202d_ddv2\\app\\app_dd"
)

set(_refreshed "")
set(_skipped "")
set(_missing "")
set(_failed "")

foreach(entry IN LISTS TARGETS)
    string(REGEX REPLACE "^([^|]+)\\|([^|]+)\\|(.*)$" "\\1" tag "${entry}")
    string(REGEX REPLACE "^([^|]+)\\|([^|]+)\\|(.*)$" "\\2" elf_check "${entry}")
    string(REGEX REPLACE "^([^|]+)\\|([^|]+)\\|(.*)$" "\\3" elf_export "${entry}")
    set(out "${OUT_DIR}/symbols_${tag}.json")
    to_windows_export_path("${out}" out_export)

    if(NOT EXISTS "${elf_check}")
        list(APPEND _missing "${tag}")
        continue()
    endif()

    if(EXISTS "${out}" AND NOT "${elf_check}" IS_NEWER_THAN "${out}")
        list(APPEND _skipped "${tag}")
        continue()
    endif()

    execute_process(
        COMMAND "${EXPORTER}" "${elf_export}" "${out_export}"
        RESULT_VARIABLE _rc
        OUTPUT_QUIET
        ERROR_VARIABLE _err
    )
    if(_rc EQUAL 0)
        list(APPEND _refreshed "${tag}")
    else()
        list(APPEND _failed "${tag}")
        message(WARNING "refresh_symbols: elf_export failed for ${tag} (rc=${_rc}): ${_err}")
    endif()
endforeach()

if(_refreshed)
    list(JOIN _refreshed " " _msg)
    message(STATUS "refresh_symbols: refreshed -> ${_msg}")
endif()
if(_skipped)
    list(JOIN _skipped " " _msg)
    message(STATUS "refresh_symbols: up-to-date -> ${_msg}")
endif()
if(_missing)
    list(JOIN _missing " " _msg)
    message(STATUS "refresh_symbols: ELF missing (skipped) -> ${_msg}")
endif()
# _failed already produced WARNINGs above; build continues either way.
