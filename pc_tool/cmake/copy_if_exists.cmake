if(NOT DEFINED SRC OR NOT DEFINED DST)
    message(FATAL_ERROR "copy_if_exists.cmake requires SRC and DST")
endif()

if(EXISTS "${SRC}")
    get_filename_component(_dst_dir "${DST}" DIRECTORY)
    file(MAKE_DIRECTORY "${_dst_dir}")
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different "${SRC}" "${DST}"
        RESULT_VARIABLE _copy_result
    )
    if(NOT _copy_result EQUAL 0)
        message(FATAL_ERROR "failed to copy ${SRC} to ${DST}")
    endif()
endif()
