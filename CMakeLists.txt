cmake_minimum_required(VERSION 3.19)
project(slits)

set(CMAKE_CXX_STANDARD 20)

add_executable(slits main.cpp tracy/TracyClient.cpp)

target_link_libraries(slits PUBLIC SDL2 dl)

target_compile_definitions(slits PUBLIC TRACY_ENABLE)

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()