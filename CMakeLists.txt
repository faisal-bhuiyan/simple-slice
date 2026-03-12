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

# Mesh library
add_library(mesh
    src/mesh/stl_io.cpp
    src/mesh/triangle_mesh.cpp
    src/mesh/reorient_triangles.cpp
    src/mesh/void_detection.cpp
)
target_include_directories(mesh PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_options(mesh PRIVATE ${PROJECT_WARNINGS})
add_library(simple_slice::mesh ALIAS mesh)

# Polyline library
add_library(polyline src/polyline/polyline.cpp)
target_include_directories(polyline PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_compile_options(polyline PRIVATE ${PROJECT_WARNINGS})
add_library(simple_slice::polyline ALIAS polyline)

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

add_executable(slicer_main
    src/apps/slicer_main.cpp
)
target_link_libraries(slicer_main PRIVATE simpleslice_geometry)
target_include_directories(slicer_main PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)

# Tests
include(CTest)
if(BUILD_TESTING)
    add_subdirectory(tests)
endif()
