#include <cmath>
#include <fstream>
#include <string>

#include <gtest/gtest.h>

#include "mesh/geometry.hpp"
#include "mesh/triangle_mesh.hpp"

using simple_slice::mesh::Edge;
using simple_slice::mesh::kBoundaryTriangleIndex;
using simple_slice::mesh::make_edge;
using simple_slice::mesh::Point;
using simple_slice::mesh::Triangle;
using simple_slice::mesh::TriangleMesh;

//---------------------------------------------------------------------------
// Helpers
//---------------------------------------------------------------------------

static const char* kTestStlPath = "triangle_mesh_test.stl";

/// Build a TriangleMesh from in-memory STL content (writes to a temp file)
static TriangleMesh make_mesh_from_stl(const std::string& stl_content) {
    std::ofstream f(kTestStlPath);
    if (!f) {
        throw std::runtime_error("failed to open " + std::string(kTestStlPath) + " for writing");
    }
    f << stl_content;
    f.close();
    return TriangleMesh(kTestStlPath);
}

static void expect_point_eq(const Point& actual, const Point& expected) {
    EXPECT_DOUBLE_EQ(actual[0], expected[0]);
    EXPECT_DOUBLE_EQ(actual[1], expected[1]);
    EXPECT_DOUBLE_EQ(actual[2], expected[2]);
}

//---------------------------------------------------------------------------
// Constructor — file open failure
//---------------------------------------------------------------------------

TEST(TriangleMeshConstructor, NonexistentPathThrows) {
    EXPECT_THROW(
        { TriangleMesh mesh("nonexistent_file_that_does_not_exist.stl"); }, std::invalid_argument
    );
}

//---------------------------------------------------------------------------
// Constructor — empty mesh
//---------------------------------------------------------------------------

TEST(TriangleMeshConstructor, EmptyMeshThrows) {
    // STL with no triangles
    const std::string stl = R"(
        solid empty
        endsolid empty
    )";

    std::ofstream f(kTestStlPath);
    ASSERT_TRUE(f) << "failed to create " << kTestStlPath;
    f << stl;
    f.close();

    EXPECT_THROW(
        { TriangleMesh mesh(kTestStlPath); }, std::invalid_argument
    );  // throws invalid_argument
}

//---------------------------------------------------------------------------
// Constructor — degenerate triangles
//---------------------------------------------------------------------------

TEST(TriangleMeshConstructor, DegenerateDuplicateVerticesThrows) {
    // Triangle with a == b
    const std::string stl = R"(
        solid degenerate
        facet normal 0 0 1
            outer loop
            vertex 0 0 0
            vertex 0 0 0
            vertex 0 1 0
            endloop
        endfacet
        endsolid degenerate
    )";

    EXPECT_THROW({ make_mesh_from_stl(stl); }, std::invalid_argument);  // throws invalid_argument
}

TEST(TriangleMeshConstructor, DegenerateCollinearVerticesThrows) {
    // Three collinear points (zero area)
    const std::string stl = R"(
        solid degenerate
        facet normal 0 0 1
            outer loop
            vertex 0 0 0
            vertex 1 0 0
            vertex 2 0 0
            endloop
        endfacet
        endsolid degenerate
    )";

    EXPECT_THROW({ make_mesh_from_stl(stl); }, std::invalid_argument);  // throws invalid_argument
}

//---------------------------------------------------------------------------
// Constructor — non-manifold
//---------------------------------------------------------------------------

TEST(TriangleMeshConstructor, NonManifoldEdgeSharedByThreeTrianglesThrows) {
    // Three triangles all sharing the edge (0,0,0)-(1,0,0)
    const std::string stl = R"(
        solid nonmanifold
        facet normal 0 0 1
            outer loop
            vertex 0 0 0
            vertex 1 0 0
            vertex 0.5 1 0
            endloop
        endfacet
        facet normal 0 0 1
            outer loop
            vertex 0 0 0
            vertex 1 0 0
            vertex 0.5 0.5 0
            endloop
        endfacet
        facet normal 0 0 1
            outer loop
            vertex 0 0 0
            vertex 1 0 0
            vertex 0.5 -0.5 0
            endloop
        endfacet
        endsolid nonmanifold
    )";

    EXPECT_THROW({ make_mesh_from_stl(stl); }, std::invalid_argument);  // throws invalid argument
}

//---------------------------------------------------------------------------
// Constructor — valid manifold mesh
//---------------------------------------------------------------------------

TEST(TriangleMeshConstructor, ValidManifoldTwoTrianglesSucceeds) {
    const std::string stl = R"(
        solid two_tri
        facet normal 0 0 1
            outer loop
            vertex 0 0 0
            vertex 1 0 0
            vertex 0 1 0
            endloop
        endfacet
        facet normal 0 0 1
            outer loop
            vertex 1 0 0
            vertex 1 1 0
            vertex 0 1 0
            endloop
        endfacet
        endsolid two_tri
    )";

    EXPECT_NO_THROW({
        TriangleMesh mesh = make_mesh_from_stl(stl);
        EXPECT_EQ(mesh.GetTriangles().size(), 2u);  // two triangles
    });
}

//---------------------------------------------------------------------------
// GetTriangles
//---------------------------------------------------------------------------

TEST(TriangleMeshGetTriangles, ReturnsParsedTriangles) {
    // STL with a single triangle -> returns the single triangle
    const std::string stl = R"(
        solid one
        facet normal 0 0 1
            outer loop
            vertex 0 0 0
            vertex 1 0 0
            vertex 0 1 0
            endloop
        endfacet
        endsolid one
    )";

    TriangleMesh mesh = make_mesh_from_stl(stl);
    const auto& triangles = mesh.GetTriangles();
    ASSERT_EQ(triangles.size(), 1u);
    expect_point_eq(triangles[0].a, {0., 0., 0.});
    expect_point_eq(triangles[0].b, {1., 0., 0.});
    expect_point_eq(triangles[0].c, {0., 1., 0.});
}

//---------------------------------------------------------------------------
// GetEdgeConnectivity / BuildEdgeToTriangleConnectivity
//---------------------------------------------------------------------------

TEST(TriangleMeshEdgeConnectivity, SingleTriangleAllEdgesBoundary) {
    // STL with a single triangle -> all edges are boundary edges

    /** Geometry (top view, z = 0):
     *
     *          (0,1)
     *           ●
     *          /  \
     *         / T0 \
     *        /      \
     *  (0,0) ●───────● (1,0)
     *
     * Triangle:
     *   T0: (0,0,0) -> (1,0,0) -> (0,1,0)
     *
     * Edges (all boundary edges):
     *   (0,0,0) <-> (1,0,0)
     *   (1,0,0) <-> (0,1,0)
     *   (0,1,0) <-> (0,0,0)
     *
     * Expectation:
     *   Each edge appears exactly once in the mesh,
     *   so edge connectivity = { 0, kBoundaryTriangleIndex } for all edges
     */

    const std::string stl = R"(
        solid one
        facet normal 0 0 1
            outer loop
            vertex 0 0 0
            vertex 1 0 0
            vertex 0 1 0
            endloop
        endfacet
        endsolid one
    )";
    TriangleMesh mesh = make_mesh_from_stl(stl);
    const auto& connectivity = mesh.GetEdgeConnectivity();
    EXPECT_EQ(connectivity.size(), 3u);  // three edges

    Point a{0., 0., 0.}, b{1., 0., 0.}, c{0., 1., 0.};
    Edge e_ab = make_edge(a, b);
    Edge e_bc = make_edge(b, c);
    Edge e_ca = make_edge(c, a);

    auto it_ab = connectivity.find(e_ab);
    auto it_bc = connectivity.find(e_bc);
    auto it_ca = connectivity.find(e_ca);
    ASSERT_NE(it_ab, connectivity.end());  // edge found
    ASSERT_NE(it_bc, connectivity.end());  // edge found
    ASSERT_NE(it_ca, connectivity.end());  // edge found

    // Each edge has one triangle (index 0), second slot is boundary index
    EXPECT_EQ(it_ab->second[0], 0);                       // triangle index 0
    EXPECT_EQ(it_ab->second[1], kBoundaryTriangleIndex);  // boundary index
    EXPECT_EQ(it_bc->second[0], 0);                       // triangle index 0
    EXPECT_EQ(it_bc->second[1], kBoundaryTriangleIndex);  // boundary index
    EXPECT_EQ(it_ca->second[0], 0);                       // triangle index 0
    EXPECT_EQ(it_ca->second[1], kBoundaryTriangleIndex);  // boundary index
}

TEST(TriangleMeshEdgeConnectivity, TwoTrianglesSharedEdgeHasTwoIndices) {
    // STL with two triangles sharing an edge -> edge has two indices

    /* Geometry (top view, z = 0):
     *
     *   (0,1) ●──────● (1,1)
     *         │    / │
     *          T0/T1 │
     *         │ /    │
     *   (0,0) ●──────● (1,0)
     *
     * Triangles:
     *   T0: (0,0,0) -> (1,1,0) -> (0,1,0)
     *   T1: (0,0,0) -> (1,1,0) -> (1,0,0)
     *
     * Shared edge (interior):
     *   (0,0,0) <-> (1,1,0)   -> two triangle indices
     *
     * Expectation:
     *   The shared edge appears in the edge connectivity map
     *   with two triangle indices (0 and 1).
     */

    const std::string stl = R"(
        solid two
        facet normal 0 0 1
            outer loop
            vertex 0 0 0
            vertex 1 1 0
            vertex 0 1 0
            endloop
        endfacet
        facet normal 0 0 1
            outer loop
            vertex 0 0 0
            vertex 1 1 0
            vertex 1 0 0
            endloop
        endfacet
        endsolid two
    )";
    TriangleMesh mesh = make_mesh_from_stl(stl);
    const auto& connectivity = mesh.GetEdgeConnectivity();

    // Shared edge (0,0,0)-(1,1,0) in canonical form is (0,0,0), (1,1,0)
    Point p00{0., 0., 0.}, p11{1., 1., 0.};
    Edge shared_edge = make_edge(p00, p11);
    auto it = connectivity.find(shared_edge);

    ASSERT_NE(it, connectivity.end());                // edge found
    const auto& adjacency = it->second;               // adjacency array for the edge
    EXPECT_NE(adjacency[0], kBoundaryTriangleIndex);  // not boundary index
    EXPECT_NE(adjacency[1], kBoundaryTriangleIndex);  // not boundary index
    // One index is 0, the other is 1 -> edge is shared by two triangles
    EXPECT_TRUE(
        (adjacency[0] == 0 && adjacency[1] == 1) || (adjacency[0] == 1 && adjacency[1] == 0)
    );
}

TEST(TriangleMeshEdgeConnectivity, TwoTrianglesBoundaryEdgesHaveBoundaryIndex) {
    // STL with two triangles sharing a boundary edge -> edge has two indices

    /* Geometry (top view, z = 0):
     *
     *   (0,1) ●──────● (1,1)
     *         │    / │
     *          T0/T1 │
     *         │ /    │
     *   (0,0) ●──────● (1,0)
     *
     * Triangles:
     *   T0: (0,0,0) -> (1,1,0) -> (0,1,0)
     *   T1: (0,0,0) -> (1,1,0) -> (1,0,0)
     *
     * Shared edge (interior):
     *   (0,0,0) <-> (1,1,0)   -> two triangle indices
     *
     * Boundary edges (appear in only one triangle):
     *   T0 boundary: (0,0,0) <-> (0,1,0)
     *   T0 boundary: (0,1,0) <-> (1,1,0)
     *   T1 boundary: (0,0,0) <-> (1,0,0)
     *   T1 boundary: (1,0,0) <-> (1,1,0)
     *
     * Expectation:
     *   For a boundary edge like (0,0,0) <-> (1,0,0),
     *   adjacency = { triangle_index, kBoundaryTriangleIndex }.
     */

    const std::string stl = R"(
        solid two
        facet normal 0 0 1
            outer loop
            vertex 0 0 0
            vertex 1 1 0
            vertex 0 1 0
            endloop
        endfacet
        facet normal 0 0 1
            outer loop
            vertex 0 0 0
            vertex 1 1 0
            vertex 1 0 0
            endloop
        endfacet
        endsolid two
    )";

    TriangleMesh mesh = make_mesh_from_stl(stl);
    const auto& connectivity = mesh.GetEdgeConnectivity();

    // Boundary edge on first triangle: (0,0,0)-(0,1,0)
    Edge e_00_01 = make_edge({0., 0., 0.}, {0., 1., 0.});
    auto it = connectivity.find(e_00_01);
    ASSERT_NE(it, connectivity.end());
    EXPECT_EQ(it->second[0], 0);                       // triangle index 0
    EXPECT_EQ(it->second[1], kBoundaryTriangleIndex);  // boundary index
}
