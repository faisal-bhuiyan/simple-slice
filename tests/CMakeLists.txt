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
    geometry/test_vector.cpp
    geometry/test_utilities.cpp
    geometry/test_projections.cpp
    air_hockey_game/test_air_hockey_game.cpp
    slicer/test_shapes.cpp
    slicer/test_perimeters.cpp
    slicer/test_toolpath.cpp
    slicer/test_stl_reader.cpp
    slicer/test_mesh_slicer.cpp
)

target_link_libraries(unit_tests PRIVATE
    simple_slice::geometry
    air_hockey_game_lib
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(unit_tests)
