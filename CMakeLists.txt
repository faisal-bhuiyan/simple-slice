cmake_minimum_required(VERSION 3.20)

project(SimpleSlice LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Header-only geometry library target
add_library(simpleslice_geometry INTERFACE)
target_include_directories(simpleslice_geometry INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
add_library(simple_slice::geometry ALIAS simpleslice_geometry)

# Air hockey game library target
add_library(air_hockey_game_lib
    src/air_hockey_game/air_hockey_game.cpp
)
target_include_directories(air_hockey_game_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(air_hockey_game_lib PUBLIC simple_slice::geometry)
add_library(simple_slice::air_hockey_game ALIAS air_hockey_game_lib)

# Executables
add_executable(air_hockey_game
    src/air_hockey_game/air_hockey_game_main.cpp
    src/air_hockey_game/air_hockey_game.cpp
)
target_link_libraries(air_hockey_game PRIVATE simpleslice_geometry)
target_include_directories(air_hockey_game PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

add_executable(slicer2d_main
    src/apps/slicer2d_main.cpp
)
target_link_libraries(slicer2d_main PRIVATE simpleslice_geometry)
target_include_directories(slicer2d_main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Tests
include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
