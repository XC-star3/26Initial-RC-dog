# CMakeLists.txt for DogMotor

# Add module to include path
target_include_directories(xr PUBLIC ${CMAKE_CURRENT_LIST_DIR})

# Auto-include source files
file(GLOB MODULE_DOGMOTOR_SRC CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/*.c"
)

if(MODULE_DOGMOTOR_SRC)
    target_sources(xr PRIVATE ${MODULE_DOGMOTOR_SRC})
endif()
