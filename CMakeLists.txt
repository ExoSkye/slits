cmake_minimum_required(VERSION 3.19)
project(slits CUDA CXX)

set(CMAKE_CXX_STANDARD 20)

add_executable(slits main.cpp tracy/TracyClient.cpp)
set_source_files_properties(tracy/TracyClient.cpp PROPERTIES LANGUAGE CUDA)
set_source_files_properties(main.cpp PROPERTIES LANGUAGE CUDA)

target_link_libraries(slits PUBLIC SDL2 dl cuda)

target_compile_definitions(slits PUBLIC TRACY_ENABLE)

option(fastmath "Whether to use cuda fast math or not" OFF)

if(fastmath MATCHES "ON")
    target_compile_options(slits PUBLIC -use_fast_math)
    message("Using fast math")
endif()
set_target_properties(
        slits
        PROPERTIES
        CUDA_SEPARABLE_COMPILATION ON)
set_property(TARGET
        slits
        PROPERTY
        CUDA_ARCHITECTURES 72)