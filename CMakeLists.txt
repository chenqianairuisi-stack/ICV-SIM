cmake_minimum_required(VERSION 3.21)
project(ICV_SIM VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

option(ICV_BUILD_NEVC "Build the executable linked to the installed SimOne SDK" ON)
set(SIMONE_SDK_ROOT "E:/Sim-One/SimOneAPI" CACHE PATH "Installed SimOneAPI directory (contains include and lib)")

add_library(icv_core STATIC cpp/src/control.cpp)
target_include_directories(icv_core PUBLIC cpp/include)
set_target_properties(icv_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_library(icv_native SHARED cpp/src/c_api.cpp)
target_link_libraries(icv_native PRIVATE icv_core)
target_compile_definitions(icv_native PRIVATE ICV_NATIVE_EXPORTS)

add_executable(icv_core_tests cpp/tests/core_tests.cpp)
target_link_libraries(icv_core_tests PRIVATE icv_core)
if(MSVC)
    foreach(target icv_core icv_native icv_core_tests)
        target_compile_options(${target} PRIVATE /W4 /utf-8 /permissive-)
    endforeach()
endif()

include(CTest)
if(BUILD_TESTING)
    add_test(NAME cpp_core COMMAND icv_core_tests)
endif()

find_package(Python3 COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
    add_custom_target(run_cpp_offline
        COMMAND "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/run_offline.py"
            --backend cpp --native-library "$<TARGET_FILE:icv_native>"
        DEPENDS icv_native WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}" USES_TERMINAL)
    if(BUILD_TESTING)
        add_test(NAME python_cpp_parity
            COMMAND "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/tests/test_native.py"
                "$<TARGET_FILE:icv_native>")
        set_tests_properties(python_cpp_parity PROPERTIES WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
    endif()
endif()

if(ICV_BUILD_NEVC)
    if(NOT WIN32 OR NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
        message(FATAL_ERROR "The configured SimOne SDK target requires Windows x64")
    endif()
    if(NOT EXISTS "${SIMONE_SDK_ROOT}/include/SimOneServiceAPI.h"
       OR NOT EXISTS "${SIMONE_SDK_ROOT}/lib/Win64/SimOneAPI.lib")
        message(FATAL_ERROR "Set SIMONE_SDK_ROOT to the installed SimOneAPI directory, or use the vs2022-core preset without ICV_BUILD_NEVC")
    endif()
    add_executable(icv_nevc cpp/src/nevc_main.cpp)
    target_include_directories(icv_nevc SYSTEM PRIVATE "${SIMONE_SDK_ROOT}/include")
    target_link_libraries(icv_nevc PRIVATE icv_core "${SIMONE_SDK_ROOT}/lib/Win64/SimOneAPI.lib")
    target_compile_definitions(icv_nevc PRIVATE NOMINMAX _USE_MATH_DEFINES)
    target_compile_options(icv_nevc PRIVATE /W4 /utf-8 /permissive-)
    file(GLOB sdk_runtime_dlls "${SIMONE_SDK_ROOT}/lib/Win64/*.dll")
    add_custom_command(TARGET icv_nevc POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${sdk_runtime_dlls} "$<TARGET_FILE_DIR:icv_nevc>"
        COMMENT "Copy the locally installed SDK runtime DLLs next to icv_nevc")
    if(BUILD_TESTING)
        add_test(NAME sdk_version_probe COMMAND icv_nevc --probe)
    endif()
endif()

set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT icv_core_tests)
