#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

#include "mesh/stl_io.hpp"

using simple_slice::mesh::convert_binary_stl_to_ascii;
using simple_slice::mesh::parse_ascii_stl;
using simple_slice::mesh::Point;
using simple_slice::mesh::Triangle;
using simple_slice::mesh::write_ascii_stl;

//---------------------------------------------------------------------------
// Helpers
//---------------------------------------------------------------------------

/// Parse an ASCII STL string and return the resulting triangles
static std::vector<Triangle> parse(const std::string& stl_text) {
    std::istringstream stream(stl_text);
    return parse_ascii_stl(stream);
}

/// Check that two points are equal within a tolerance
static void expect_point_eq(const Point& actual, const Point& expected) {
    EXPECT_DOUBLE_EQ(actual[0], expected[0]);
    EXPECT_DOUBLE_EQ(actual[1], expected[1]);
    EXPECT_DOUBLE_EQ(actual[2], expected[2]);
}

//---------------------------------------------------------------------------
// Empty / degenerate input
//---------------------------------------------------------------------------

TEST(ParseAsciiStl, EmptyInputReturnsNoTriangles) {
    auto triangles = parse("");
    EXPECT_TRUE(triangles.empty());
}

TEST(ParseAsciiStl, NoVertexKeywordsReturnsNoTriangles) {
    auto triangles = parse("solid cube\nendsolid cube\n");
    EXPECT_TRUE(triangles.empty());
}

//---------------------------------------------------------------------------
// Single triangle
//---------------------------------------------------------------------------

TEST(ParseAsciiStl, SingleTriangle) {
    // STL with a single triangle -> returns the single triangle
    const std::string stl = R"(
        solid single
          facet normal 0 0 1
            outer loop
              vertex 0.0 0.0 0.0
              vertex 1.0 0.0 0.0
              vertex 0.0 1.0 0.0
            endloop
          endfacet
        endsolid single
    )";

    auto triangles = parse(stl);
    ASSERT_EQ(triangles.size(), 1u);
    expect_point_eq(triangles[0].a, {0., 0., 0.});
    expect_point_eq(triangles[0].b, {1., 0., 0.});
    expect_point_eq(triangles[0].c, {0., 1., 0.});
}

//---------------------------------------------------------------------------
// Multiple triangles
//---------------------------------------------------------------------------

TEST(ParseAsciiStl, TwoTriangles) {
    // STL with two triangles -> returns the two triangles
    const std::string stl = R"(
        solid two
          facet normal 0 0 1
            outer loop
              vertex 0 0 0
              vertex 1 0 0
              vertex 0 1 0
            endloop
          endfacet
          facet normal 0 0 -1
            outer loop
              vertex 1 0 0
              vertex 1 1 0
              vertex 0 1 0
            endloop
          endfacet
        endsolid two
    )";

    auto triangles = parse(stl);
    ASSERT_EQ(triangles.size(), 2u);

    expect_point_eq(triangles[0].a, {0, 0, 0});
    expect_point_eq(triangles[0].b, {1, 0, 0});
    expect_point_eq(triangles[0].c, {0, 1, 0});

    expect_point_eq(triangles[1].a, {1, 0, 0});
    expect_point_eq(triangles[1].b, {1, 1, 0});
    expect_point_eq(triangles[1].c, {0, 1, 0});
}

//---------------------------------------------------------------------------
// Negative coordinates
//---------------------------------------------------------------------------

TEST(ParseAsciiStl, NegativeCoordinates) {
    // STL with negative coordinates -> returns the triangle with negative coordinates
    const std::string stl = R"(
        solid neg
          facet normal 0 0 1
            outer loop
              vertex -1.5 -2.5 -3.5
              vertex  4.0  5.0  6.0
              vertex -7.0  8.0 -9.0
            endloop
          endfacet
        endsolid neg
    )";

    auto triangles = parse(stl);
    ASSERT_EQ(triangles.size(), 1u);
    expect_point_eq(triangles[0].a, {-1.5, -2.5, -3.5});
    expect_point_eq(triangles[0].b, {4., 5., 6.});
    expect_point_eq(triangles[0].c, {-7., 8., -9.});
}

//---------------------------------------------------------------------------
// Scientific notation
//---------------------------------------------------------------------------

TEST(ParseAsciiStl, ScientificNotation) {
    // STL with scientific notation -> returns the triangle with scientific notation
    const std::string stl = R"(
        solid sci
          facet normal 0 0 1
            outer loop
              vertex 1.5e2 -3.0e-1 0.0e0
              vertex 1e3 2e-4 3e+1
              vertex -1.23e+2 4.56e-3 7.89e0
            endloop
          endfacet
        endsolid sci
    )";
    auto triangles = parse(stl);
    ASSERT_EQ(triangles.size(), 1u);
    expect_point_eq(triangles[0].a, {150., -0.3, 0.});
    expect_point_eq(triangles[0].b, {1000., 0.0002, 30.});
    expect_point_eq(triangles[0].c, {-123., 0.00456, 7.89});
}

//---------------------------------------------------------------------------
// Whitespace variations
//---------------------------------------------------------------------------

TEST(ParseAsciiStl, ExtraWhitespaceAndTabs) {
    // STL with tabs, extra spaces, blank lines -> returns the triangle with the whitespace
    const std::string stl =
        "solid ws\n"
        "\n"
        "  facet normal 0 0 1\n"
        "    outer loop\n"
        "\t\tvertex\t\t1.0   2.0   3.0\n"
        "      vertex   4.0\t5.0\t6.0\n"
        "\n"
        "      vertex 7.0 8.0 9.0\n"
        "    endloop\n"
        "  endfacet\n"
        "endsolid ws\n";

    auto triangles = parse(stl);
    ASSERT_EQ(triangles.size(), 1u);
    expect_point_eq(triangles[0].a, {1., 2., 3.});
    expect_point_eq(triangles[0].b, {4., 5., 6.});
    expect_point_eq(triangles[0].c, {7., 8., 9.});
}

//---------------------------------------------------------------------------
// Minimal (no header/footer keywords)
//---------------------------------------------------------------------------

TEST(ParseAsciiStl, MinimalVertexOnlyInput) {
    // STL with minimal vertex only input -> returns the triangle with the minimal vertex only input
    const std::string stl =
        "vertex 0 0 0\n"
        "vertex 1 0 0\n"
        "vertex 0 1 0\n";

    auto triangles = parse(stl);
    ASSERT_EQ(triangles.size(), 1u);
    expect_point_eq(triangles[0].a, {0, 0, 0});
    expect_point_eq(triangles[0].b, {1, 0, 0});
    expect_point_eq(triangles[0].c, {0, 1, 0});
}

//---------------------------------------------------------------------------
// Incomplete triangle (only 2 vertices)
//---------------------------------------------------------------------------

TEST(ParseAsciiStl, IncompleteTriangleIsDropped) {
    // STL with incomplete triangle -> drops the incomplete triangle
    const std::string stl = R"(
        solid incomplete
          facet normal 0 0 1
            outer loop
              vertex 1 2 3
              vertex 4 5 6
            endloop
          endfacet
        endsolid incomplete
    )";

    auto triangles = parse(stl);
    EXPECT_TRUE(triangles.empty());
}

//---------------------------------------------------------------------------
// Mixed: one complete + one incomplete triangle
//---------------------------------------------------------------------------

TEST(ParseAsciiStl, CompleteTriangleFollowedByIncomplete) {
    // STL with complete triangle followed by incomplete triangle -> returns the complete triangle
    const std::string stl = R"(
        solid mixed
          facet normal 0 0 1
            outer loop
              vertex 0 0 0
              vertex 1 0 0
              vertex 0 1 0
            endloop
          endfacet
          facet normal 0 0 1
            outer loop
              vertex 2 2 2
              vertex 3 3 3
            endloop
          endfacet
        endsolid mixed
    )";

    auto triangles = parse(stl);
    ASSERT_EQ(triangles.size(), 1u);
    expect_point_eq(triangles[0].a, {0, 0, 0});
}

//---------------------------------------------------------------------------
// Large coordinate values
//---------------------------------------------------------------------------

TEST(ParseAsciiStl, LargeCoordinateValues) {
    // STL with large coordinate values -> returns the triangle with large coordinate values
    const std::string stl = R"(
        solid large
          facet normal 0 0 1
            outer loop
              vertex 999999.999 -999999.999 0.000001
              vertex 123456789.0 0.0 0.0
              vertex 0.0 987654321.0 0.0
            endloop
          endfacet
        endsolid large
    )";

    auto triangles = parse(stl);
    ASSERT_EQ(triangles.size(), 1u);
    expect_point_eq(triangles[0].a, {999999.999, -999999.999, 0.000001});
    expect_point_eq(triangles[0].b, {123456789., 0., 0.});
    expect_point_eq(triangles[0].c, {0., 987654321., 0.});
}

//---------------------------------------------------------------------------
// Integer coordinates (no decimal point)
//---------------------------------------------------------------------------

TEST(ParseAsciiStl, IntegerCoordinates) {
    // STL with integer coordinates -> returns the triangle with integer coordinates
    const std::string stl = R"(
        solid ints
          facet normal 0 0 1
            outer loop
              vertex 1 2 3
              vertex 4 5 6
              vertex 7 8 9
            endloop
          endfacet
        endsolid ints
    )";

    auto triangles = parse(stl);
    ASSERT_EQ(triangles.size(), 1u);
    expect_point_eq(triangles[0].a, {1, 2, 3});
    expect_point_eq(triangles[0].b, {4, 5, 6});
    expect_point_eq(triangles[0].c, {7, 8, 9});
}

//---------------------------------------------------------------------------
// Write tests
//---------------------------------------------------------------------------

TEST(WriteAsciiStl, SingleTriangleRoundTrip) {
    // STL with a single triangle -> returns the single triangle
    Triangle t{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};

    std::ostringstream out;
    write_ascii_stl(out, "single", std::span<const Triangle>{&t, 1});

    auto parsed = parse(out.str());
    ASSERT_EQ(parsed.size(), 1u);
    expect_point_eq(parsed[0].a, t.a);
    expect_point_eq(parsed[0].b, t.b);
    expect_point_eq(parsed[0].c, t.c);
}

TEST(WriteAsciiStl, MultipleTrianglesRoundTrip) {
    // STL with multiple triangles -> returns the multiple triangles
    std::vector<Triangle> tris{
        {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}},
        {{1, 0, 0}, {1, 1, 0}, {0, 1, 0}},
    };

    std::ostringstream out;
    write_ascii_stl(out, "two", tris);

    auto parsed = parse(out.str());
    ASSERT_EQ(parsed.size(), tris.size());
}

TEST(WriteAsciiStl, DegenerateTriangleZeroArea) {
    // STL with degenerate triangle -> returns the degenerate triangle
    Triangle t{{1, 1, 1}, {1, 1, 1}, {1, 1, 1}};

    std::ostringstream out;
    write_ascii_stl(out, "degenerate", std::span<const Triangle>{&t, 1});

    auto parsed = parse(out.str());
    ASSERT_EQ(parsed.size(), 1u);
}

TEST(WriteAsciiStl, NegativeAndLargeCoordinates) {
    // STL with negative and large coordinates -> returns the triangle with negative and large
    // coordinates
    Triangle t{{-1e6, 2.5, -3.5}, {4., -5e5, 6.}, {7., 8., -9e4}};

    std::ostringstream out;
    write_ascii_stl(out, "coords", std::span<const Triangle>{&t, 1});

    auto parsed = parse(out.str());
    ASSERT_EQ(parsed.size(), 1u);
}

TEST(WriteAsciiStl, EmptyTriangleList) {
    // STL with empty triangle list -> returns the empty triangle list
    std::ostringstream out;
    write_ascii_stl(out, "empty", {});

    EXPECT_EQ(out.str(), "solid empty\nendsolid empty\n");
}

TEST(WriteAsciiStl, DeterministicOutput) {
    // STL with deterministic output -> returns the deterministic output
    Triangle t{{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};

    std::ostringstream out1, out2;
    write_ascii_stl(out1, "det", std::span<const Triangle>{&t, 1});
    write_ascii_stl(out2, "det", std::span<const Triangle>{&t, 1});

    EXPECT_EQ(out1.str(), out2.str());
}

//---------------------------------------------------------------------------
// Convert binary STL to ASCII
//---------------------------------------------------------------------------

/// Write a minimal binary STL file (80-byte header + little-endian uint32_t count + triangles)
/// Each triangle: 12 bytes normal (3 float), 36 bytes vertices (9 float), 2 bytes attribute
static void write_minimal_binary_stl(
    const std::string& path, std::uint32_t num_triangles,
    const float*
        vertices_per_triangle  // 9 floats per triangle (v0x,v0y,v0z, v1x,v1y,v1z, v2x,v2y,v2z)
) {
    std::ofstream out(path, std::ios::binary);
    ASSERT_TRUE(out) << "failed to open " << path;
    char header[80] = {};
    out.write(header, 80);
    out.write(reinterpret_cast<const char*>(&num_triangles), sizeof(num_triangles));
    float normal[3] = {0.f, 0.f, 1.f};
    std::uint16_t attr = 0;
    for (std::uint32_t i = 0; i < num_triangles; ++i) {
        out.write(reinterpret_cast<const char*>(normal), sizeof(normal));
        out.write(reinterpret_cast<const char*>(vertices_per_triangle + i * 9), 9 * sizeof(float));
        out.write(reinterpret_cast<const char*>(&attr), sizeof(attr));
    }
}

TEST(ConvertBinaryStlToAscii, NonexistentBinaryPathThrows) {
    EXPECT_THROW(
        convert_binary_stl_to_ascii("nonexistent_binary.stl", "out.stl"), std::runtime_error
    );
}

TEST(ConvertBinaryStlToAscii, ZeroTrianglesProducesValidAscii) {
    const std::string binary_path = "binary_zero_triangles.stl";
    const std::string ascii_path = "ascii_zero_triangles.stl";
    write_minimal_binary_stl(binary_path, 0u, nullptr);

    // Convert binary STL to ASCII
    convert_binary_stl_to_ascii(binary_path, ascii_path);
    std::ifstream in(ascii_path);
    auto triangles = parse_ascii_stl(in);
    EXPECT_TRUE(triangles.empty());
}

TEST(ConvertBinaryStlToAscii, OneTriangleRoundTrip) {
    const std::string binary_path = "binary_one_triangle.stl";
    const std::string ascii_path = "ascii_one_triangle.stl";
    const float verts[9] = {
        0.f, 0.f, 0.f,  // triangle 0, vertex 0
        1.f, 0.f, 0.f,  // triangle 0, vertex 1
        0.f, 1.f, 0.f,  // triangle 0, vertex 2
    };
    write_minimal_binary_stl(binary_path, 1u, verts);

    // Convert binary STL to ASCII
    convert_binary_stl_to_ascii(binary_path, ascii_path);
    std::ifstream in(ascii_path);
    auto triangles = parse_ascii_stl(in);
    ASSERT_EQ(triangles.size(), 1u);
    expect_point_eq(triangles[0].a, {0., 0., 0.});
    expect_point_eq(triangles[0].b, {1., 0., 0.});
    expect_point_eq(triangles[0].c, {0., 1., 0.});
}

TEST(ConvertBinaryStlToAscii, TwoTrianglesRoundTrip) {
    const std::string binary_path = "binary_two_triangles.stl";
    const std::string ascii_path = "ascii_two_triangles.stl";
    const float verts[18] = {
        0.f, 0.f, 0.f,  // triangle 0, vertex 0
        1.f, 0.f, 0.f,  // triangle 0, vertex 1
        0.f, 1.f, 0.f,  // triangle 0, vertex 2
        0.f, 0.f, 1.f,  // triangle 1, vertex 0
        0.f, 1.f, 1.f,  // triangle 1, vertex 1
        1.f, 0.f, 1.f,  // triangle 1, vertex 2
    };
    write_minimal_binary_stl(binary_path, 2u, verts);

    // Convert binary STL to ASCII
    convert_binary_stl_to_ascii(binary_path, ascii_path);
    std::ifstream in(ascii_path);
    auto triangles = parse_ascii_stl(in);
    ASSERT_EQ(triangles.size(), 2u);
    expect_point_eq(triangles[0].a, {0., 0., 0.});
    expect_point_eq(triangles[0].b, {1., 0., 0.});
    expect_point_eq(triangles[0].c, {0., 1., 0.});
    expect_point_eq(triangles[1].a, {0., 0., 1.});
    expect_point_eq(triangles[1].b, {0., 1., 1.});
    expect_point_eq(triangles[1].c, {1., 0., 1.});
}
