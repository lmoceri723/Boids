cmake_minimum_required(VERSION 3.11)
project(MyRaylibProject C)

cmake_policy(SET CMP0048 NEW)
cmake_policy(SET CMP0077 NEW)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(OpenMP QUIET)

include(FetchContent)
FetchContent_Declare(
        raylib
        GIT_REPOSITORY https://github.com/raysan5/raylib.git
        GIT_TAG 5.0
)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(RAYLIB_BUILD_UTILS OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(raylib)

FetchContent_Declare(
        raygui
        GIT_REPOSITORY https://github.com/raysan5/raygui.git
        GIT_TAG 4.0
)
FetchContent_MakeAvailable(raygui)

message(STATUS "Raylib source directory: ${raylib_SOURCE_DIR}")
message(STATUS "Raygui source directory: ${raygui_SOURCE_DIR}")

add_executable(MyRaylibProject
        main.c
        simulation.c
        gui.c
        utils.c
)

target_link_libraries(MyRaylibProject PRIVATE raylib)

if(OpenMP_FOUND)
    message(STATUS "OpenMP found. Attempting to enable OpenMP for MyRaylibProject.")
    if(MSVC)
        target_compile_options(MyRaylibProject PRIVATE "$<$<COMPILE_LANGUAGE:C>:/openmp:llvm>")
    else()
        target_link_libraries(MyRaylibProject PRIVATE OpenMP::OpenMP_C)
    endif()
else()
    message(WARNING "OpenMP not found. The project will be built without OpenMP parallelization. "
            "Install OpenMP runtime libraries and ensure your compiler supports it (e.g., libomp-dev and -fopenmp for GCC/Clang, or /openmp for MSVC).")
endif()

target_include_directories(MyRaylibProject PRIVATE
        ${raylib_SOURCE_DIR}/src
        ${raygui_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}
)

if(MSVC)
    set_target_properties(MyRaylibProject PROPERTIES LINK_FLAGS "/SUBSYSTEM:WINDOWS /ENTRY:mainCRTStartup")
    target_compile_options(MyRaylibProject PRIVATE /W3 /wd4996)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)