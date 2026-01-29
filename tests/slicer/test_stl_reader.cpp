#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "slicer/stl_reader.hpp"

using namespace simple_slice::slicer;

// Test fixture for STL reader tests
class STLReaderTest : public ::testing::Test {
protected:
    static constexpr double kTestEpsilon = 1e-9;

    // Helper to create a simple STL string
    std::string CreateSimpleSTL() {
        return R"(solid cube
  facet normal 0 0 1
    outer loop
      vertex 0 0 0
      vertex 1 0 0
      vertex 1 1 0
    endloop
  endfacet
endsolid cube)";
    }

    // Helper to create STL with multiple triangles
    std::string CreateMultiTriangleSTL() {
        return R"(solid test
  facet normal 0 0 1
    outer loop
      vertex 0 0 0
      vertex 1 0 0
      vertex 1 1 0
    endloop
  endfacet
  facet normal 0 0 1
    outer loop
      vertex 0 0 0
      vertex 1 1 0
      vertex 0 1 0
    endloop
  endfacet
endsolid test)";
    }
};

//----------------------------------------------
// Parse ASCII STL tests
//----------------------------------------------

TEST_F(STLReaderTest, ParseSimpleTriangle) {
    std::istringstream input(CreateSimpleSTL());
    const auto triangles = parse_ascii_stl(input);

    ASSERT_EQ(triangles.size(), 1U);
    EXPECT_NEAR(triangles[0].a.GetX(), 0., kTestEpsilon);
    EXPECT_NEAR(triangles[0].a.GetY(), 0., kTestEpsilon);
    EXPECT_NEAR(triangles[0].a.GetZ(), 0., kTestEpsilon);
    EXPECT_NEAR(triangles[0].b.GetX(), 1., kTestEpsilon);
    EXPECT_NEAR(triangles[0].b.GetY(), 0., kTestEpsilon);
    EXPECT_NEAR(triangles[0].b.GetZ(), 0., kTestEpsilon);
    EXPECT_NEAR(triangles[0].c.GetX(), 1., kTestEpsilon);
    EXPECT_NEAR(triangles[0].c.GetY(), 1., kTestEpsilon);
    EXPECT_NEAR(triangles[0].c.GetZ(), 0., kTestEpsilon);
}

TEST_F(STLReaderTest, ParseMultipleTriangles) {
    std::istringstream input(CreateMultiTriangleSTL());
    const auto triangles = parse_ascii_stl(input);

    ASSERT_EQ(triangles.size(), 2U);

    // First triangle
    EXPECT_NEAR(triangles[0].a.GetX(), 0., kTestEpsilon);
    EXPECT_NEAR(triangles[0].a.GetY(), 0., kTestEpsilon);
    EXPECT_NEAR(triangles[0].b.GetX(), 1., kTestEpsilon);
    EXPECT_NEAR(triangles[0].b.GetY(), 0., kTestEpsilon);
    EXPECT_NEAR(triangles[0].c.GetX(), 1., kTestEpsilon);
    EXPECT_NEAR(triangles[0].c.GetY(), 1., kTestEpsilon);

    // Second triangle
    EXPECT_NEAR(triangles[1].a.GetX(), 0., kTestEpsilon);
    EXPECT_NEAR(triangles[1].a.GetY(), 0., kTestEpsilon);
    EXPECT_NEAR(triangles[1].b.GetX(), 1., kTestEpsilon);
    EXPECT_NEAR(triangles[1].b.GetY(), 1., kTestEpsilon);
    EXPECT_NEAR(triangles[1].c.GetX(), 0., kTestEpsilon);
    EXPECT_NEAR(triangles[1].c.GetY(), 1., kTestEpsilon);
}

TEST_F(STLReaderTest, ParseEmptySTL) {
    std::istringstream input("");
    const auto triangles = parse_ascii_stl(input);

    EXPECT_TRUE(triangles.empty());
}

TEST_F(STLReaderTest, ParseSTLWithNoVertices) {
    std::istringstream input("solid test\nendsolid test");
    const auto triangles = parse_ascii_stl(input);

    EXPECT_TRUE(triangles.empty());
}

TEST_F(STLReaderTest, ParseSTLWith3DCoordinates) {
    std::istringstream input(R"(solid test
  facet normal 0 0 1
    outer loop
      vertex 0 0 0
      vertex 1 0 1
      vertex 0 1 2
    endloop
  endfacet
endsolid test)");
    const auto triangles = parse_ascii_stl(input);

    ASSERT_EQ(triangles.size(), 1U);
    EXPECT_NEAR(triangles[0].a.GetZ(), 0., kTestEpsilon);
    EXPECT_NEAR(triangles[0].b.GetZ(), 1., kTestEpsilon);
    EXPECT_NEAR(triangles[0].c.GetZ(), 2., kTestEpsilon);
}

TEST_F(STLReaderTest, ParseSTLWithNegativeCoordinates) {
    std::istringstream input(R"(solid test
  facet normal 0 0 1
    outer loop
      vertex -1 -2 -3
      vertex -4 -5 -6
      vertex -7 -8 -9
    endloop
  endfacet
endsolid test)");
    const auto triangles = parse_ascii_stl(input);

    ASSERT_EQ(triangles.size(), 1U);
    EXPECT_NEAR(triangles[0].a.GetX(), -1., kTestEpsilon);
    EXPECT_NEAR(triangles[0].a.GetY(), -2., kTestEpsilon);
    EXPECT_NEAR(triangles[0].a.GetZ(), -3., kTestEpsilon);
}

TEST_F(STLReaderTest, ParseSTLWithDecimalCoordinates) {
    std::istringstream input(R"(solid test
  facet normal 0 0 1
    outer loop
      vertex 0.5 1.25 2.75
      vertex 3.14 2.71 1.41
      vertex 0.1 0.2 0.3
    endloop
  endfacet
endsolid test)");
    const auto triangles = parse_ascii_stl(input);

    ASSERT_EQ(triangles.size(), 1U);
    EXPECT_NEAR(triangles[0].a.GetX(), 0.5, kTestEpsilon);
    EXPECT_NEAR(triangles[0].a.GetY(), 1.25, kTestEpsilon);
    EXPECT_NEAR(triangles[0].b.GetX(), 3.14, kTestEpsilon);
    EXPECT_NEAR(triangles[0].b.GetY(), 2.71, kTestEpsilon);
}

TEST_F(STLReaderTest, ParseSTLMalformedIncompleteVertex) {
    std::istringstream input(R"(solid test
  facet normal 0 0 1
    outer loop
      vertex 0 0 0
      vertex 1 0
      vertex 1 1 0
    endloop
  endfacet
endsolid test)");
    const auto triangles = parse_ascii_stl(input);

    // When vertex parsing fails (missing z coordinate), the stream goes into error state
    // and parsing stops, so no triangles are returned
    EXPECT_EQ(triangles.size(), 0U);
}

TEST_F(STLReaderTest, ParseSTLWithExtraVertices) {
    std::istringstream input(R"(solid test
  facet normal 0 0 1
    outer loop
      vertex 0 0 0
      vertex 1 0 0
      vertex 1 1 0
      vertex 0 1 0
    endloop
  endfacet
endsolid test)");
    const auto triangles = parse_ascii_stl(input);

    // Should parse first 3 vertices as one triangle, then next 3 as another
    ASSERT_EQ(triangles.size(), 1U);
}

TEST_F(STLReaderTest, ParseSTLWithWhitespace) {
    std::istringstream input(R"(solid   test
  facet   normal   0   0   1
    outer   loop
      vertex   0   0   0
      vertex   1   0   0
      vertex   1   1   0
    endloop
  endfacet
endsolid   test)");
    const auto triangles = parse_ascii_stl(input);

    ASSERT_EQ(triangles.size(), 1U);
    EXPECT_NEAR(triangles[0].a.GetX(), 0., kTestEpsilon);
    EXPECT_NEAR(triangles[0].b.GetX(), 1., kTestEpsilon);
}

//----------------------------------------------
// Read ASCII STL file tests
//----------------------------------------------

TEST_F(STLReaderTest, ReadNonExistentFile) {
    const auto triangles = read_ascii_stl_file("nonexistent_file.stl");

    EXPECT_TRUE(triangles.empty());
}

TEST_F(STLReaderTest, ReadValidFile) {
    // Create a temporary file for testing
    const std::string test_file = "test_stl_reader_temp.stl";
    std::ofstream file(test_file);
    ASSERT_TRUE(file.is_open());
    file << CreateSimpleSTL();
    file.close();

    const auto triangles = read_ascii_stl_file(test_file);

    ASSERT_EQ(triangles.size(), 1U);
    EXPECT_NEAR(triangles[0].a.GetX(), 0., kTestEpsilon);

    // Clean up
    std::remove(test_file.c_str());
}
