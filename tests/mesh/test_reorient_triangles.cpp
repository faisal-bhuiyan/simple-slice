#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>

#if defined(_WIN32)
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <gtest/gtest.h>

#include "mesh/geometry.hpp"
#include "mesh/reorient_triangles.hpp"
#include "mesh/stl_io.hpp"
#include "mesh/triangle_mesh.hpp"

using simple_slice::mesh::are_orientations_consistent;
using simple_slice::mesh::Edge;
using simple_slice::mesh::flip_triangle;
using simple_slice::mesh::has_directed_edge;
using simple_slice::mesh::make_edge;
using simple_slice::mesh::parse_ascii_stl;
using simple_slice::mesh::Point;
using simple_slice::mesh::reorient_inconsistent_triangles;
using simple_slice::mesh::Triangle;
using simple_slice::mesh::TriangleMesh;
using simple_slice::mesh::write_ascii_stl;

//---------------------------------------------------------------------------
// Helpers
//---------------------------------------------------------------------------

static void expect_point_eq(const Point& actual, const Point& expected) {
    EXPECT_DOUBLE_EQ(actual[0], expected[0]);
    EXPECT_DOUBLE_EQ(actual[1], expected[1]);
    EXPECT_DOUBLE_EQ(actual[2], expected[2]);
}

/// Build a TriangleMesh from in-memory STL content (writes to a temp file).
/// Uses a process-unique filename so parallel test runs do not overwrite the same file.
static TriangleMesh make_mesh_from_stl(const std::string& stl_content) {
#if defined(_WIN32)
    const unsigned long long pid = GetCurrentProcessId();
#else
    const unsigned long long pid = static_cast<unsigned long long>(getpid());
#endif
    const std::string path = "reorient_test_" + std::to_string(pid) + ".stl";
    std::ofstream f(path);
    if (!f) {
        throw std::runtime_error("failed to open " + path + " for writing");
    }
    f << stl_content;
    f.close();
    return TriangleMesh(path);
}

/// Build STL string for a strip of 2 * num_quads triangles (each quad = 2 triangles, inconsistent)
static std::string make_stl_strip_inconsistent(std::size_t num_quads) {
    std::ostringstream stl;
    stl << "solid strip\n";
    for (std::size_t k = 0; k < num_quads; ++k) {
        double x0 = static_cast<double>(k);
        double x1 = static_cast<double>(k + 1);
        // First triangle of quad: (x0,0,0), (x1,0,0), (x0,1,0)
        stl << "  facet normal 0 0 1\n    outer loop\n"
            << "      vertex " << x0 << " 0 0\n"
            << "      vertex " << x1 << " 0 0\n"
            << "      vertex " << x0 << " 1 0\n"
            << "    endloop\n  endfacet\n";
        // Second triangle: (x1,0,0), (x0,1,0), (x1,1,0) — same orientation on shared edge ->
        // inconsistent
        stl << "  facet normal 0 0 1\n    outer loop\n"
            << "      vertex " << x1 << " 0 0\n"
            << "      vertex " << x0 << " 1 0\n"
            << "      vertex " << x1 << " 1 0\n"
            << "    endloop\n  endfacet\n";
    }
    stl << "endsolid strip\n";
    return stl.str();
}

/// Build STL string for an NxM grid of triangles, all consistently oriented
static std::string make_stl_grid_consistent(std::size_t nrows, std::size_t ncols) {
    std::ostringstream stl;
    stl << "solid grid\n";
    for (std::size_t i = 0; i < nrows; ++i) {
        for (std::size_t j = 0; j < ncols; ++j) {
            double x0 = static_cast<double>(j);
            double x1 = static_cast<double>(j + 1);
            double y0 = static_cast<double>(i);
            double y1 = static_cast<double>(i + 1);
            // Two triangles per cell, both same winding (e.g. ccw)
            stl << "  facet normal 0 0 1\n    outer loop\n"
                << "      vertex " << x0 << " " << y0 << " 0\n"
                << "      vertex " << x1 << " " << y0 << " 0\n"
                << "      vertex " << x0 << " " << y1 << " 0\n"
                << "    endloop\n  endfacet\n";
            stl << "  facet normal 0 0 1\n    outer loop\n"
                << "      vertex " << x1 << " " << y0 << " 0\n"
                << "      vertex " << x1 << " " << y1 << " 0\n"
                << "      vertex " << x0 << " " << y1 << " 0\n"
                << "    endloop\n  endfacet\n";
        }
    }
    stl << "endsolid grid\n";
    return stl.str();
}

/// Strip facets with y offset (for building disconnected multi-strip STL)
static std::string make_stl_strip_facets_at_y(std::size_t num_quads, double y_offset) {
    std::ostringstream stl;
    double y0 = y_offset;
    double y1 = y_offset + 1.0;
    for (std::size_t k = 0; k < num_quads; ++k) {
        double x0 = static_cast<double>(k);
        double x1 = static_cast<double>(k + 1);
        stl << "  facet normal 0 0 1\n    outer loop\n"
            << "      vertex " << x0 << " " << y0 << " 0\n"
            << "      vertex " << x1 << " " << y0 << " 0\n"
            << "      vertex " << x0 << " " << y1 << " 0\n"
            << "    endloop\n  endfacet\n";
        stl << "  facet normal 0 0 1\n    outer loop\n"
            << "      vertex " << x1 << " " << y0 << " 0\n"
            << "      vertex " << x0 << " " << y1 << " 0\n"
            << "      vertex " << x1 << " " << y1 << " 0\n"
            << "    endloop\n  endfacet\n";
    }
    return stl.str();
}

/// Build STL string for an NxM grid of triangles, all inconsistently oriented
/// Stacked strips so every shared edge has the same direction in both triangles
static std::string make_stl_grid_inconsistent(std::size_t nrows, std::size_t ncols) {
    std::ostringstream stl;
    stl << "solid grid_inconsistent\n";
    for (std::size_t i = 0; i < nrows; ++i) {
        stl << make_stl_strip_facets_at_y(ncols, static_cast<double>(i));
    }
    stl << "endsolid grid_inconsistent\n";
    return stl.str();
}

//---------------------------------------------------------------------------
// flip_triangle
//---------------------------------------------------------------------------

TEST(FlipTriangle, SwapsBAndC) {
    Triangle t{{0., 0., 0.}, {1., 0., 0.}, {0., 1., 0.}};
    flip_triangle(t);
    expect_point_eq(t.a, {0., 0., 0.});
    expect_point_eq(t.b, {0., 1., 0.});
    expect_point_eq(t.c, {1., 0., 0.});
}

TEST(FlipTriangle, Idempotent) {
    Triangle t{{0., 0., 0.}, {1., 0., 0.}, {0., 1., 0.}};
    flip_triangle(t);
    flip_triangle(t);  // idempotent operation
    expect_point_eq(t.a, {0., 0., 0.});
    expect_point_eq(t.b, {1., 0., 0.});
    expect_point_eq(t.c, {0., 1., 0.});
}

//---------------------------------------------------------------------------
// has_directed_edge
//---------------------------------------------------------------------------

TEST(HasDirectedEdge, DetectsDirectedEdgesAB_BC_CA) {
    // directed edge from a to b, b to c, c to a
    Point a{0., 0., 0.}, b{1., 0., 0.}, c{0., 1., 0.};
    Triangle t{a, b, c};
    EXPECT_TRUE(has_directed_edge(t, a, b));  // contains directed edge from a to b
    EXPECT_TRUE(has_directed_edge(t, b, c));  // contains directed edge from b to c
    EXPECT_TRUE(has_directed_edge(t, c, a));  // contains directed edge from c to a
}

TEST(HasDirectedEdge, RejectsReverseDirection) {
    // directed edge from a to b, b to c, c to a
    Point a{0., 0., 0.}, b{1., 0., 0.}, c{0., 1., 0.};
    Triangle t{a, b, c};
    EXPECT_FALSE(has_directed_edge(t, b, a));  // does not traverse in correct order
    EXPECT_FALSE(has_directed_edge(t, c, b));  // does not traverse in correct order
    EXPECT_FALSE(has_directed_edge(t, a, c));  // does not traverse in correct order
}

TEST(HasDirectedEdge, RejectsNonEdge) {
    // directed edge from a to b, b to c, c to a
    Point a{0., 0., 0.}, b{1., 0., 0.}, c{0., 1., 0.};
    Triangle t{a, b, c};
    Point d{2., 0., 0.};
    EXPECT_FALSE(has_directed_edge(t, a, d));  // d is not a vertex of the triangle
}

//---------------------------------------------------------------------------
// are_orientations_consistent
//---------------------------------------------------------------------------

TEST(AreOrientationsConsistent, OppositeDirectionIsConsistent) {
    Point p{0., 0., 0.}, q{1., 0., 0.}, r{0., 1., 0.};
    Triangle t1{p, q, r};  // edge (p,q)
    Triangle t2{q, p, r};  // edge (q,p) -> opposite
    Edge e = make_edge(p, q);
    EXPECT_TRUE(are_orientations_consistent(t1, t2, e));  // orientations are consistent
}

TEST(AreOrientationsConsistent, SameDirectionIsInconsistent) {
    Point p{0., 0., 0.}, q{1., 0., 0.}, r1{0., 1., 0.}, r2{1., 1., 0.};
    Triangle t1{p, q, r1};  // edge (p,q)
    Triangle t2{p, q, r2};  // edge (p,q) same direction
    Edge e = make_edge(p, q);
    EXPECT_FALSE(are_orientations_consistent(t1, t2, e));  // orientations are NOT consistent
}

//---------------------------------------------------------------------------
// reorient_inconsistent_triangles — mesh from STL
//---------------------------------------------------------------------------

TEST(ReorientInconsistentTriangles, SeedOutOfRangeReturnsEmpty) {
    // seed is out of range -> no triangles to reorient
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

    TriangleMesh mesh = make_mesh_from_stl(stl);              // single triangle
    auto flipped = reorient_inconsistent_triangles(mesh, 1);  // seed is out of range
    EXPECT_TRUE(flipped.empty());                             // no triangles to reorient
}

TEST(ReorientInconsistentTriangles, SingleTriangleReturnsEmpty) {
    // single triangle -> no triangles to reorient
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
    auto flipped = reorient_inconsistent_triangles(mesh, 0);  // seed is 0
    EXPECT_TRUE(flipped.empty());                             // no triangles to reorient
}

TEST(ReorientInconsistentTriangles, TwoTrianglesConsistentReturnsEmpty) {
    // Two triangles sharing edge (0,0,0)-(1,0,0), opposite orientation
    const std::string stl = R"(
        solid two_consistent
          facet normal 0 0 1
            outer loop
              vertex 0 0 0
              vertex 1 0 0
              vertex 0 1 0
            endloop
          endfacet
          facet normal 0 0 -1
            outer loop
              vertex 0 0 0
              vertex 0 1 0
              vertex 1 0 0
            endloop
          endfacet
        endsolid two_consistent
    )";

    TriangleMesh mesh =
        make_mesh_from_stl(stl);  // two triangles sharing edge (0,0,0)-(1,0,0), opposite orientation
    auto flipped = reorient_inconsistent_triangles(mesh, 0);  // seed is 0
    EXPECT_TRUE(flipped.empty());                             // no triangles to reorient
}

TEST(ReorientInconsistentTriangles, TwoTrianglesInconsistentReturnsOneFlipped) {
    // Two triangles sharing edge (0,0,0)-(1,0,0), same orientation -> neighbor flipped
    const std::string stl = R"(
        solid two_inconsistent
          facet normal 0 0 1
            outer loop
              vertex 0 0 0
              vertex 1 0 0
              vertex 0 1 0
            endloop
          endfacet
          facet normal 0 0 1
            outer loop
              vertex 0 0 0
              vertex 1 0 0
              vertex 1 1 0
            endloop
          endfacet
        endsolid two_inconsistent
    )";

    TriangleMesh mesh =
        make_mesh_from_stl(stl);  // two triangles sharing edge (0,0,0)-(1,0,0), same orientation
    auto flipped = reorient_inconsistent_triangles(mesh, 0);  // seed is 0
    ASSERT_EQ(flipped.size(), 1u);                            // one triangle flipped

    // Neighbor (index 1) had vertices (0,0,0), (1,0,0), (1,1,0). Flipped -> (0,0,0), (1,1,0),
    // (1,0,0)
    expect_point_eq(flipped[0].a, {0., 0., 0.});
    expect_point_eq(flipped[0].b, {1., 1., 0.});
    expect_point_eq(flipped[0].c, {1., 0., 0.});
}

TEST(ReorientInconsistentTriangles, MeshUnchangedAfterCall) {
    // two triangles sharing edge (0,0,0)-(1,0,0), same orientation -> neighbor flipped
    const std::string stl = R"(
        solid two_inconsistent
          facet normal 0 0 1
            outer loop
              vertex 0 0 0
              vertex 1 0 0
              vertex 0 1 0
            endloop
          endfacet
          facet normal 0 0 1
            outer loop
              vertex 0 0 0
              vertex 1 0 0
              vertex 1 1 0
            endloop
          endfacet
        endsolid two_inconsistent
    )";

    TriangleMesh mesh =
        make_mesh_from_stl(stl);  // two triangles sharing edge (0,0,0)-(1,0,0), same orientation
    const auto& before = mesh.GetTriangles();
    (void)reorient_inconsistent_triangles(mesh, 0);  // seed is 0
    const auto& after = mesh.GetTriangles();
    ASSERT_EQ(
        before.size(), after.size()
    );  // same number of triangles before and after reorientation

    // same vertices before and after reorientation
    for (std::size_t i = 0; i < before.size(); ++i) {
        expect_point_eq(after[i].a, before[i].a);  // same vertices
        expect_point_eq(after[i].b, before[i].b);  // same vertices
        expect_point_eq(after[i].c, before[i].c);  // same vertices
    }
}

//---------------------------------------------------------------------------
// export_inconsistent_triangles
//---------------------------------------------------------------------------

TEST(ExportInconsistentTriangles, WritesValidStlToStreamAndReadsItBack) {
    // two triangles T0 and T1 sharing edge (1,0,0)-(0,1,0): inconsistent orientation -> flip T1

    /** Geometry (top view, z = 0):
     *
     *   (0,1) ●--------● (1,1)
     *         │ \      │
     *         │  \  T1 │
     *         │   \  * │
     *         │    \   │
     *         │     \  │
     *         │ T0   \ │
     *   (0,0) ●--------● (1,0)
     *
     * Triangles (initial orientation):
     *   T0: (0,0,0) -> (1,0,0) -> (0,1,0)      [seed triangle]
     *   T1: (1,0,0) -> (0,1,0) -> (1,1,0)      [* inconsistent triangle]
     *
     * Shared edge:
     *   (1,0,0) <-> (0,1,0)
     *
     * Issue:
     *   Both triangles traverse the shared edge in the SAME direction, making T1
     *   orientation inconsistent with respect to the seed T0
     *
     * Export behavior:
     *   export_inconsistent_triangles(mesh, 0, ...)
     *   selects and flips the inconsistent neighbor (T1 *), then writes ONLY that triangle to the
     *   output STL
     *
     * Expectation:
     *   The exported STL contains exactly one triangle (T1), with vertex order flipped to restore
     *   consistent orientation
     */

    const std::string stl = R"(
        solid two_inconsistent
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
              vertex 0 1 0
              vertex 1 1 0
            endloop
          endfacet
        endsolid two_inconsistent
    )";

    // Export inconsistent triangles to stream and read them back
    TriangleMesh mesh = make_mesh_from_stl(stl);
    std::ostringstream out;
    export_inconsistent_triangles(mesh, 0, out);
    std::string s = out.str();
    EXPECT_TRUE(s.find("solid") != std::string::npos);
    EXPECT_TRUE(s.find("endsolid") != std::string::npos);
    EXPECT_TRUE(s.find("vertex") != std::string::npos);

    // Import inconsistent triangles from stream and check them
    std::istringstream in(s);
    auto triangles = parse_ascii_stl(in);

    ASSERT_EQ(triangles.size(), 1u);
    // Neighbor (index 1) had vertices (1,0,0), (0,1,0), (1,1,0)
    // Flipped -> (1,0,0), (1,1,0), (0,1,0)
    expect_point_eq(triangles[0].a, {1., 0., 0.});
    expect_point_eq(triangles[0].b, {1., 1., 0.});
    expect_point_eq(triangles[0].c, {0., 1., 0.});
}

//---------------------------------------------------------------------------
// Stress tests — time and space
//---------------------------------------------------------------------------

TEST(ReorientTrianglePerformanceTest, LargeConsistentGrid_ReturnsEmpty) {
    // Grid of 50x40 = 2000 cells -> 4000 triangles; all consistent -> no triangles to reorient

    /*
     * STL grid schematic (top view, z = 0):
     *
     *   y=2  ●--------●--------●
     *        │ \      │ \      │
     *        │  \     │  \     │
     *        │   \    │   \    │
     *        │    \   │    \   │
     *        │     \  │     \  │
     *        │      \ │      \ │
     *   y=1  ●--------●--------●
     *        │ \      │ \      │
     *        │  \     │  \     │
     *        │   \    │   \    │
     *        │    \   │    \   │
     *        │     \  │     \  │
     *        │      \ │      \ │
     *   y=0  ●--------●--------●
     *        x=0      x=1      x=2
     *
     * Each cell [i,j] is split into two triangles with consistent winding:
     *
     *   T0: (x0,y0,0) -> (x1,y0,0) -> (x0,y1,0)
     *   T1: (x1,y0,0) -> (x1,y1,0) -> (x0,y1,0)
     *
     * All triangles traverse shared edges in opposite directions,
     * producing a globally consistent, manifold mesh.
     *
     * The grid contains 2 * nrows * ncols triangles
     */

    const std::size_t rows = 50;
    const std::size_t cols = 40;
    std::string stl = make_stl_grid_consistent(rows, cols);
    TriangleMesh mesh = make_mesh_from_stl(stl);
    ASSERT_EQ(mesh.GetTriangles().size(), rows * cols * 2u);

    auto flipped = reorient_inconsistent_triangles(mesh, 0);
    EXPECT_TRUE(flipped.empty());
}

TEST(ReorientTrianglePerformanceTest, LargeInconsistentGrid_ReturnsAllButSeed) {
    // Grid of 50x40 = 2000 cells -> 4000 triangles; all inconsistent (stacked strips)

    /*
     * STL grid schematic (top view, z = 0):
     *
     *   y=2  ●--------●--------●
     *        │ \      │ \      │
     *        │  \     │  \     │
     *        │   \    │   \    │
     *        │    \   │    \   │
     *        │     \  │     \  │
     *        │      \ │      \ │
     *   y=1  ●--------●--------●
     *        │ \      │ \      │
     *        │  \     │  \     │
     *        │   \    │   \    │
     *        │    \   │    \   │
     *        │     \  │     \  │
     *        │      \ │      \ │
     *   y=0  ●--------●--------●
     *        x=0      x=1      x=2
     *
     * Each cell [i,j] is split into two triangles with inconsistent winding:
     *
     *   T0: (x0,y0,0) -> (x1,y0,0) -> (x0,y1,0)
     *   T1: (x1,y0,0) -> (x1,y1,0) -> (x0,y1,0)
     *
     * All triangles traverse shared edges in same direction,
     * producing a globally in inconsistent, manifold mesh.
     *
     * The grid contains 2 * nrows * ncols triangles
     */

    const std::size_t rows = 50;
    const std::size_t cols = 40;
    std::string stl = make_stl_grid_inconsistent(rows, cols);
    TriangleMesh mesh = make_mesh_from_stl(stl);
    ASSERT_EQ(mesh.GetTriangles().size(), rows * cols * 2u);

    auto flipped = reorient_inconsistent_triangles(mesh, 0);
    EXPECT_EQ(flipped.size(), rows * cols * 2u - 1u);

    {
        std::ofstream out("stress _grid_reoriented.stl");
        if (out) {
            write_ascii_stl(out, "stress_grid_reoriented", flipped);
        }
    }
}

TEST(ReorientTrianglePerformanceTest, LongStripAllInconsistent_ReturnsAllButSeed) {
    // Strip of 500 quads -> 1000 triangles; each pair

    /*
     * STL strip schematic (top view, z = 0):
     *
     *   y=1  ●--------●--------●--------●--------●
     *        │ \      │ \      │ \      │ \      │
     *        │  \     │  \     │  \     │  \     │
     *        │   \    │   \    │   \    │   \    │
     *        │    \   │    \   │    \   │    \   │
     *        │     \  │     \  │     \  │     \  │
     *        │      \ │      \ │      \ │      \ │
     *   y=0  ●--------●--------●--------●--------●
     *        x=0      x=1      x=2      x=3      x=4
     *
     * Each quad [k, k+1] is split into two triangles:
     *
     *   T0: (x0,0,0) -> (x1,0,0) -> (x0,1,0)
     *   T1: (x1,0,0) -> (x0,1,0) -> (x1,1,0)   *
     *
     * The two triangles in each quad traverse their shared edge
     * (x0,0,0) <-> (x1,0,0) in the SAME direction, making T1 (*)
     * orientation inconsistent with respect to T0.
     *
     * Repeating this pattern produces a strip of 2 * num_quads triangles
     * with a consistent local inconsistency in every triangle
     */

    const std::size_t num_quads = 500;
    std::string stl = make_stl_strip_inconsistent(num_quads);
    TriangleMesh mesh = make_mesh_from_stl(stl);
    ASSERT_EQ(mesh.GetTriangles().size(), num_quads * 2u);

    auto flipped = reorient_inconsistent_triangles(mesh, 0);
    EXPECT_EQ(flipped.size(), num_quads * 2u - 1u);

    {
        std::ofstream out("stress _strip_reoriented.stl");
        if (out) {
            write_ascii_stl(out, "stress_strip_reoriented", flipped);
        }
    }
}

TEST(ReorientTrianglePerformanceTest, TwoDisconnectedComponents_OnlySeedComponentProcessed) {
    // Two strips: first 3 quads (6 triangles), second 2 quads (4 triangles). Disconnected.

    /*
     * STL strip schematic (top view, z = 0):
     *
     *   y=1  ●--------●--------●--------●--------●
     *        │ \      │ \      │ \      │ \      │
     *        │  \     │  \     │  \     │  \     │
     *        │   \    │   \    │   \    │   \    │
     *        │    \   │    \   │    \   │    \   │
     *        │     \  │     \  │     \  │     \  │
     *        │      \ │      \ │      \ │      \ │
     *   y=0  ●--------●--------●--------●--------●
     *        x=0      x=1      x=2      x=3      x=4
     *
     * Each quad [k, k+1] is split into two triangles:
     *
     *   T0: (x0,0,0) -> (x1,0,0) -> (x0,1,0)
     *   T1: (x1,0,0) -> (x0,1,0) -> (x1,1,0)   *
     *
     * The two triangles in each quad traverse their shared edge
     * (x0,0,0) <-> (x1,0,0) in the SAME direction, making T1 (*)
     * orientation inconsistent with respect to T0.
     *
     * Repeating this pattern produces a strip of 2 * num_quads triangles
     * with a consistent local inconsistency in every triangle
     */

    std::string stl1 = make_stl_strip_inconsistent(3);
    std::string stl2 = make_stl_strip_inconsistent(2);

    // Offset second strip so it doesn't touch the first (e.g. y += 10)
    std::ostringstream combined;
    combined << "solid two\n"
             << stl1.substr(stl1.find("facet"), stl1.rfind("endfacet") - stl1.find("facet") + 9);
    combined << "  facet normal 0 0 1\n    outer loop\n      vertex 0 10 0\n      vertex 1 10 0\n   "
                "   vertex 0 11 0\n    endloop\n  endfacet\n";
    combined << "  facet normal 0 0 1\n    outer loop\n      vertex 1 10 0\n      vertex 0 11 0\n   "
                "   vertex 1 11 0\n    endloop\n  endfacet\n";
    combined << "  facet normal 0 0 1\n    outer loop\n      vertex 0 11 0\n      vertex 1 11 0\n   "
                "   vertex 0 12 0\n    endloop\n  endfacet\n";
    combined << "  facet normal 0 0 1\n    outer loop\n      vertex 1 11 0\n      vertex 0 12 0\n   "
                "   vertex 1 12 0\n    endloop\n  endfacet\n";
    combined << "endsolid two\n";
    TriangleMesh mesh = make_mesh_from_stl(combined.str());

    // Seed 0 in first component (6 triangles)
    // second component has 4 triangles, unreachable from seed
    auto flipped = reorient_inconsistent_triangles(mesh, 0);

    // First component: 6 triangles, 5 flipped (all but seed)
    EXPECT_EQ(flipped.size(), 5u);

    {
        std::ofstream out("stress _strip_reoriented.stl");
        if (out) {
            write_ascii_stl(out, "stress_strip_reoriented", flipped);
        }
    }
}

TEST(ReorientTrianglePerformanceTest, FiveDisconnectedComponents_OnlySeedComponentProcessed) {
    /**
     * 5 strips at y = 0, 10, 20, 30, 40. Sizes: 2, 2, 100, 2, 2 quads (4, 4, 200, 4, 4 triangles)
     *
     * Seed in the large one (third component); only its 199 flipped triangles are returned
     */

    /*
     * STL strip schematic (top view, z = 0):
     *
     *   y=1  ●--------●--------●--------●--------●
     *        │ \      │ \      │ \      │ \      │
     *        │  \     │  \     │  \     │  \     │
     *        │   \    │   \    │   \    │   \    │
     *        │    \   │    \   │    \   │    \   │
     *        │     \  │     \  │     \  │     \  │
     *        │      \ │      \ │      \ │      \ │
     *   y=0  ●--------●--------●--------●--------●
     *        x=0      x=1      x=2      x=3      x=4
     *
     * Each quad [k, k+1] is split into two triangles:
     *
     *   T0: (x0,0,0) -> (x1,0,0) -> (x0,1,0)
     *   T1: (x1,0,0) -> (x0,1,0) -> (x1,1,0)   *
     *
     * The two triangles in each quad traverse their shared edge
     * (x0,0,0) <-> (x1,0,0) in the SAME direction, making T1 (*)
     * orientation inconsistent with respect to T0.
     *
     * Repeating this pattern produces a strip of 2 * num_quads triangles
     * with a consistent local inconsistency in every triangle
     */

    const std::size_t small_quads = 2;
    const std::size_t large_quads = 100;
    std::ostringstream combined;
    combined << "solid five\n";
    combined << make_stl_strip_facets_at_y(small_quads, 0.);   // 4 triangles
    combined << make_stl_strip_facets_at_y(small_quads, 10.);  // 4 triangles
    combined << make_stl_strip_facets_at_y(
        large_quads, 20.
    );  // 200 triangles (contains seed triangle)
    combined << make_stl_strip_facets_at_y(small_quads, 30.);  // 4 triangles
    combined << make_stl_strip_facets_at_y(small_quads, 40.);  // 4 triangles
    combined << "endsolid five\n";

    TriangleMesh mesh = make_mesh_from_stl(combined.str());
    const std::size_t total_tri = 4u + 4u + 200u + 4u + 4u;
    ASSERT_EQ(mesh.GetTriangles().size(), total_tri);

    // Seed = first triangle of the large (third) component: 4 + 4 = 8
    const std::size_t seed = 4u + 4u;
    auto flipped = reorient_inconsistent_triangles(mesh, seed);

    // Only the large component is processed: 200 - 1 = 199 flipped
    EXPECT_EQ(flipped.size(), large_quads * 2u - 1u);

    {
        std::ofstream out("stress _strip_reoriented.stl");
        if (out) {
            write_ascii_stl(out, "stress_strip_reoriented", flipped);
        }
    }
}
