set(LIBXR_SYSTEM FreeRTOS)
set(LIBXR_DRIVER st)
set(XROBOT_MODULES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Modules)
add_subdirectory(Middlewares/Third_Party/LibXR)

target_link_libraries(xr PUBLIC stm32cubemx)
target_compile_features(xr PUBLIC cxx_std_20)
target_include_directories(xr
    PUBLIC
        Core/Inc
        User
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE User)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE User/app_main.cpp)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE xr)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Og)
    target_compile_options(xr PRIVATE -O2)
    if(TARGET FreeRTOS)
        target_compile_options(FreeRTOS PRIVATE -O2)
    endif()

    if(TARGET STM32_Drivers)
        target_compile_options(STM32_Drivers PRIVATE -O2)
    endif()
endif()
