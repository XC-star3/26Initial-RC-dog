function(rcdog_verify_generated repository_root manifest_path)
    if(NOT EXISTS "${manifest_path}")
        message(FATAL_ERROR
            "Generated-code manifest is missing: ${manifest_path}\n"
            "Run: python3 tools/generate.py")
    endif()

    include("${manifest_path}")

    foreach(kind IN ITEMS INPUT OUTPUT)
        set(paths "${RCDOG_CODEGEN_${kind}_PATHS}")
        set(hashes "${RCDOG_CODEGEN_${kind}_SHA256}")
        list(LENGTH paths path_count)
        list(LENGTH hashes hash_count)
        if(NOT path_count EQUAL hash_count OR path_count EQUAL 0)
            message(FATAL_ERROR "Invalid generated-code manifest ${kind} entries")
        endif()

        math(EXPR last_index "${path_count} - 1")
        foreach(index RANGE 0 ${last_index})
            list(GET paths ${index} relative_path)
            list(GET hashes ${index} expected_hash)
            set(absolute_path "${repository_root}/${relative_path}")
            if(NOT EXISTS "${absolute_path}")
                message(FATAL_ERROR
                    "Generated-code dependency is missing: ${relative_path}\n"
                    "Run: python3 tools/generate.py")
            endif()
            file(SHA256 "${absolute_path}" actual_hash)
            if(NOT actual_hash STREQUAL expected_hash)
                message(FATAL_ERROR
                    "Generated code is stale: ${relative_path}\n"
                    "Run: python3 tools/generate.py")
            endif()
        endforeach()
    endforeach()

    message(STATUS
        "Generated platform verified: libxr ${RCDOG_CODEGEN_LIBXR_VERSION}, "
        "xrobot ${RCDOG_CODEGEN_XROBOT_VERSION}")
endfunction()

if(DEFINED RCDOG_VERIFY_ROOT AND DEFINED RCDOG_VERIFY_MANIFEST)
    rcdog_verify_generated("${RCDOG_VERIFY_ROOT}" "${RCDOG_VERIFY_MANIFEST}")
endif()
