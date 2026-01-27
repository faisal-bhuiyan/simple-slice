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
    test_vector.cpp
    test_utilities.cpp
    test_projections.cpp
)

target_link_libraries(unit_tests PRIVATE
    simple_slice::geometry
    GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(unit_tests)
