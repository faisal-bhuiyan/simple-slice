include(FetchContent)

FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)

set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# One executable for all unit tests
add_executable(unit_tests
    air_hockey_game/test_air_hockey_game.cpp
    geometry/test_vector.cpp
    geometry/test_utilities.cpp
    geometry/test_projections.cpp
    mesh/test_geometry.cpp
    mesh/test_reorient_triangles.cpp
    mesh/test_stl_io.cpp
    mesh/test_triangle_mesh.cpp
    mesh/test_void_detection.cpp
    polyline/test_polyline.cpp
    slicer/test_shapes.cpp
    slicer/test_perimeters.cpp
    slicer/test_toolpath.cpp
    slicer/test_stl_reader.cpp
    slicer/test_mesh_slicer.cpp
)

target_link_libraries(unit_tests PRIVATE
    air_hockey_game_lib
    simple_slice::geometry
    simple_slice::polyline
    simple_slice::mesh
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(unit_tests)
