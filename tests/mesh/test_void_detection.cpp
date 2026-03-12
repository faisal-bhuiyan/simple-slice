#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "mesh/geometry.hpp"
#include "mesh/stl_io.hpp"
#include "mesh/triangle_mesh.hpp"
#include "mesh/void_detection.hpp"

using simple_slice::mesh::AxisAlignedBoundingBox;
using simple_slice::mesh::compute_component_aabb;
using simple_slice::mesh::ConnectedComponent;
using simple_slice::mesh::convert_binary_stl_to_ascii;
using simple_slice::mesh::export_voids_to_stl;
using simple_slice::mesh::find_connected_components;
using simple_slice::mesh::identify_voids;
using simple_slice::mesh::is_connected_component_closed;
using simple_slice::mesh::TriangleMesh;

//---------------------------------------------------------------------------
// Helpers
//---------------------------------------------------------------------------

static const char* kVoidTestStlPath = "void_detection_test.stl";

static TriangleMesh make_mesh_from_stl(const std::string& stl_content) {
    std::ofstream f(kVoidTestStlPath);
    if (!f) {
        throw std::runtime_error("failed to open " + std::string(kVoidTestStlPath) + " for writing");
    }
    f << stl_content;
    f.close();
    return TriangleMesh(kVoidTestStlPath);
}

/// Closed cube [0,0,0]-[1,1,1], 12 triangles (6 faces × 2)
static std::string make_cube_stl() {
    return R"(
        solid cube
          facet normal 0 0 -1
            outer loop
              vertex 0 0 0
              vertex 1 0 0
              vertex 1 1 0
            endloop
          endfacet
          facet normal 0 0 -1
            outer loop
              vertex 0 0 0
              vertex 1 1 0
              vertex 0 1 0
            endloop
          endfacet
          facet normal 0 0 1
            outer loop
              vertex 0 0 1
              vertex 0 1 1
              vertex 1 1 1
            endloop
          endfacet
          facet normal 0 0 1
            outer loop
              vertex 0 0 1
              vertex 1 1 1
              vertex 1 0 1
            endloop
          endfacet
          facet normal 0 -1 0
            outer loop
              vertex 0 0 0
              vertex 0 0 1
              vertex 1 0 1
            endloop
          endfacet
          facet normal 0 -1 0
            outer loop
              vertex 0 0 0
              vertex 1 0 1
              vertex 1 0 0
            endloop
          endfacet
          facet normal 0 1 0
            outer loop
              vertex 0 1 0
              vertex 1 1 0
              vertex 1 1 1
            endloop
          endfacet
          facet normal 0 1 0
            outer loop
              vertex 0 1 0
              vertex 1 1 1
              vertex 0 1 1
            endloop
          endfacet
          facet normal -1 0 0
            outer loop
              vertex 0 0 0
              vertex 0 1 0
              vertex 0 1 1
            endloop
          endfacet
          facet normal -1 0 0
            outer loop
              vertex 0 0 0
              vertex 0 1 1
              vertex 0 0 1
            endloop
          endfacet
          facet normal 1 0 0
            outer loop
              vertex 1 0 0
              vertex 1 0 1
              vertex 1 1 1
            endloop
          endfacet
          facet normal 1 0 0
            outer loop
              vertex 1 0 0
              vertex 1 1 1
              vertex 1 1 0
            endloop
          endfacet
        endsolid cube
    )";
}

/// Two disjoint closed cubes: one at [0,0,0]-[1,1,1], one at [2,2,2]-[3,3,3]
static std::string make_two_disjoint_cubes_stl() {
    std::string s = make_cube_stl();
    const std::string endsolid = "endsolid cube";
    const auto pos = s.rfind(endsolid);
    assert(pos != std::string::npos);
    s.resize(pos);
    // Trim trailing newline
    while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) {
        s.pop_back();
    }
    s += R"(
        facet normal 0 0 -1
          outer loop
            vertex 2 2 2
            vertex 3 2 2
            vertex 3 3 2
          endloop
        endfacet
        facet normal 0 0 -1
          outer loop
            vertex 2 2 2
            vertex 3 3 2
            vertex 2 3 2
          endloop
        endfacet
        facet normal 0 0 1
          outer loop
            vertex 2 2 3
            vertex 2 3 3
            vertex 3 3 3
          endloop
        endfacet
        facet normal 0 0 1
          outer loop
            vertex 2 2 3
            vertex 3 3 3
            vertex 3 2 3
          endloop
        endfacet
        facet normal 0 -1 0
          outer loop
            vertex 2 2 2
            vertex 2 2 3
            vertex 3 2 3
          endloop
        endfacet
        facet normal 0 -1 0
          outer loop
            vertex 2 2 2
            vertex 3 2 3
            vertex 3 2 2
          endloop
        endfacet
        facet normal 0 1 0
          outer loop
            vertex 2 3 2
            vertex 3 3 2
            vertex 3 3 3
          endloop
        endfacet
        facet normal 0 1 0
          outer loop
            vertex 2 3 2
            vertex 3 3 3
            vertex 2 3 3
          endloop
        endfacet
        facet normal -1 0 0
          outer loop
            vertex 2 2 2
            vertex 2 3 2
            vertex 2 3 3
          endloop
        endfacet
        facet normal -1 0 0
          outer loop
            vertex 2 2 2
            vertex 2 3 3
            vertex 2 2 3
          endloop
        endfacet
        facet normal 1 0 0
          outer loop
            vertex 3 2 2
            vertex 3 2 3
            vertex 3 3 3
          endloop
        endfacet
        facet normal 1 0 0
          outer loop
            vertex 3 2 2
            vertex 3 3 3
            vertex 3 3 2
          endloop
        endfacet
      endsolid two_cubes
    )";
    return s;
}

/// Outer cube [0,0,0]-[2,2,2] and inner cube [0.5,0.5,0.5]-[1.5,1.5,1.5] (disjoint in mesh, inner
/// AABB contained in outer)
static std::string make_outer_inner_cubes_stl() {
    std::string s;
    s += R"(
        solid outer_inner
    )";
    // Outer cube 0..2 (scale 2 from unit cube)
    auto emit_cube = [&s](double x0, double y0, double z0, double x1, double y1, double z1) {
        s += "  facet normal 0 0 -1\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z0) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 0 -1\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " +
             std::to_string(x0) + " " + std::to_string(y1) + " " + std::to_string(z0) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 0 1\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " + std::to_string(x0) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 0 1\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y0) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 -1 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x0) +
             " " + std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y0) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 -1 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y0) + " " + std::to_string(z0) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 1 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 1 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x0) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal -1 0 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x0) +
             " " + std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " +
             std::to_string(x0) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal -1 0 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x0) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x0) + " " + std::to_string(y0) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 1 0 0\n    outer loop\n      vertex " + std::to_string(x1) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 1 0 0\n    outer loop\n      vertex " + std::to_string(x1) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z0) +
             "\n    endloop\n  endfacet\n";
    };
    emit_cube(0, 0, 0, 2, 2, 2);
    emit_cube(0.5, 0.5, 0.5, 1.5, 1.5, 1.5);
    s += "endsolid outer_inner\n";
    return s;
}

/// Outer cube [0,0,0]-[2,2,2] and inner closed tetrahedron (4 facets) inside [0.5,1.5]^3
static std::string make_outer_cube_inner_tetrahedron_stl() {
    std::string s;
    s += R"(
        solid outer_cube_inner_tetrahedron
    )";
    auto emit_cube = [&s](double x0, double y0, double z0, double x1, double y1, double z1) {
        s += "  facet normal 0 0 -1\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z0) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 0 -1\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " +
             std::to_string(x0) + " " + std::to_string(y1) + " " + std::to_string(z0) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 0 1\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " + std::to_string(x0) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 0 1\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y0) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 -1 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x0) +
             " " + std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y0) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 -1 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y0) + " " + std::to_string(z0) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 1 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 1 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x0) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal -1 0 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x0) +
             " " + std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " +
             std::to_string(x0) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal -1 0 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x0) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x0) + " " + std::to_string(y0) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 1 0 0\n    outer loop\n      vertex " + std::to_string(x1) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 1 0 0\n    outer loop\n      vertex " + std::to_string(x1) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z0) +
             "\n    endloop\n  endfacet\n";
    };
    emit_cube(0, 0, 0, 2, 2, 2);

    // Tetrahedron: base z=0.5, apex (1,1,1.5). Vertices (0.5,0.5,0.5), (1.5,0.5,0.5), (1,1.5,0.5),
    // (1,1,1.5)
    s += R"(
          facet normal 0 0 -1
            outer loop
              vertex 0.5 0.5 0.5
              vertex 1 1.5 0.5
              vertex 1.5 0.5 0.5
            endloop
          endfacet
          facet normal 0.816497 -0.408248 -0.408248
            outer loop
              vertex 0.5 0.5 0.5
              vertex 1.5 0.5 0.5
              vertex 1 1 1.5
            endloop
          endfacet
          facet normal -0.408248 0.816497 -0.408248
            outer loop
              vertex 0.5 0.5 0.5
              vertex 1 1 1.5
              vertex 1 1.5 0.5
            endloop
          endfacet
          facet normal -0.408248 -0.408248 0.816497
            outer loop
              vertex 1.5 0.5 0.5
              vertex 1 1.5 0.5
              vertex 1 1 1.5
            endloop
          endfacet
        endsolid outer_cube_inner_tetrahedron
    )";
    return s;
}

/// Outer cube [0,0,0]-[2,2,2] and inner closed pyramid (square base, 6 facets)
static std::string make_outer_cube_inner_pyramid_stl() {
    std::string s;
    s += R"(
        solid outer_cube_inner_pyramid
    )";
    auto emit_cube = [&s](double x0, double y0, double z0, double x1, double y1, double z1) {
        s += "  facet normal 0 0 -1\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z0) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 0 -1\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " +
             std::to_string(x0) + " " + std::to_string(y1) + " " + std::to_string(z0) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 0 1\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " + std::to_string(x0) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 0 1\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y0) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 -1 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x0) +
             " " + std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y0) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 -1 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y0) + " " + std::to_string(z0) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 1 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 1 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x0) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal -1 0 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x0) +
             " " + std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " +
             std::to_string(x0) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal -1 0 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x0) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x0) + " " + std::to_string(y0) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 1 0 0\n    outer loop\n      vertex " + std::to_string(x1) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 1 0 0\n    outer loop\n      vertex " + std::to_string(x1) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z0) +
             "\n    endloop\n  endfacet\n";
    };
    emit_cube(0, 0, 0, 2, 2, 2);

    // Pyramid: base z=0.5 (0.5,0.5,0.5)-(1.5,1.5,0.5), apex (1,1,1.5)
    s += R"(
          facet normal 0 0 -1
            outer loop
              vertex 0.5 0.5 0.5
              vertex 1.5 0.5 0.5
              vertex 1.5 1.5 0.5
            endloop
          endfacet
          facet normal 0 0 -1
            outer loop
              vertex 0.5 0.5 0.5
              vertex 1.5 1.5 0.5
              vertex 0.5 1.5 0.5
            endloop
          endfacet
          facet normal 0 1 0
            outer loop
              vertex 0.5 0.5 0.5
              vertex 1.5 0.5 0.5
              vertex 1 1 1.5
            endloop
          endfacet
          facet normal 1 0 0
            outer loop
              vertex 1.5 0.5 0.5
              vertex 1.5 1.5 0.5
              vertex 1 1 1.5
            endloop
          endfacet
          facet normal 0 -1 0
            outer loop
              vertex 1.5 1.5 0.5
              vertex 0.5 1.5 0.5
              vertex 1 1 1.5
            endloop
          endfacet
          facet normal -1 0 0
            outer loop
              vertex 0.5 1.5 0.5
              vertex 0.5 0.5 0.5
              vertex 1 1 1.5
            endloop
          endfacet
        endsolid outer_cube_inner_pyramid
    )";
    return s;
}

//---------------------------------------------------------------------------
// find_connected_components
//---------------------------------------------------------------------------

TEST(FindConnectedComponents, SingleTriangleReturnsOneComponent) {
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
    auto components = find_connected_components(mesh);
    ASSERT_EQ(components.size(), 1u);
    EXPECT_EQ(components[0].size(), 1u);
}

TEST(FindConnectedComponents, OneCubeReturnsOneComponent) {
    TriangleMesh mesh = make_mesh_from_stl(make_cube_stl());
    auto components = find_connected_components(mesh);
    ASSERT_EQ(components.size(), 1u);
    EXPECT_EQ(components[0].size(), 12u);
}

TEST(FindConnectedComponents, TwoDisjointCubesReturnsTwoComponents) {
    TriangleMesh mesh = make_mesh_from_stl(make_two_disjoint_cubes_stl());
    auto components = find_connected_components(mesh);
    ASSERT_EQ(components.size(), 2u);
    EXPECT_EQ(components[0].size(), 12u);
    EXPECT_EQ(components[1].size(), 12u);
}

//---------------------------------------------------------------------------
// is_connected_component_closed
//---------------------------------------------------------------------------

TEST(IsConnectedComponentClosed, SingleTriangleIsOpen) {
    // Single triangle is open
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
    auto components = find_connected_components(mesh);
    ASSERT_EQ(components.size(), 1u);
    EXPECT_FALSE(is_connected_component_closed(mesh, components[0]));
}

TEST(IsConnectedComponentClosed, OneCubeIsClosed) {
    // One cube is closed
    TriangleMesh mesh = make_mesh_from_stl(make_cube_stl());
    auto components = find_connected_components(mesh);
    ASSERT_EQ(components.size(), 1u);
    EXPECT_EQ(components[0].size(), 12u);
    EXPECT_TRUE(is_connected_component_closed(mesh, components[0]));
}

//---------------------------------------------------------------------------
// compute_component_aabb
//---------------------------------------------------------------------------

TEST(ComputeComponentAabb, OneCubeHasExpectedExtents) {
    TriangleMesh mesh = make_mesh_from_stl(make_cube_stl());
    auto components = find_connected_components(mesh);
    ASSERT_EQ(components.size(), 1u);
    AxisAlignedBoundingBox box = compute_component_aabb(mesh, components[0], 0.);
    EXPECT_DOUBLE_EQ(box.min_x, 0.);
    EXPECT_DOUBLE_EQ(box.min_y, 0.);
    EXPECT_DOUBLE_EQ(box.min_z, 0.);
    EXPECT_DOUBLE_EQ(box.max_x, 1.);
    EXPECT_DOUBLE_EQ(box.max_y, 1.);
    EXPECT_DOUBLE_EQ(box.max_z, 1.);
}

//---------------------------------------------------------------------------
// identify_voids
//---------------------------------------------------------------------------

TEST(IdentifyVoids, SingleClosedCubeReturnsNoVoids) {
    TriangleMesh mesh = make_mesh_from_stl(make_cube_stl());
    auto components = find_connected_components(mesh);
    std::vector<ConnectedComponent> closed;
    for (const auto& c : components) {
        if (is_connected_component_closed(mesh, c)) {
            closed.push_back(std::move(c));
        }
    }

    ASSERT_EQ(closed.size(), 1u);  // one closed component
    // one closed component: single closed cube
    auto voids = identify_voids(mesh, closed);
    EXPECT_TRUE(voids.empty());  // no voids
}

TEST(IdentifyVoids, InnerCubeContainedInOuterCubeIsVoid) {
    TriangleMesh mesh = make_mesh_from_stl(make_outer_inner_cubes_stl());
    auto components = find_connected_components(mesh);
    std::vector<ConnectedComponent> closed;
    for (const auto& c : components) {
        if (is_connected_component_closed(mesh, c)) {
            closed.push_back(std::move(c));
        }
    }

    ASSERT_EQ(closed.size(), 2u);  // two closed components
    // two closed components: outer cube and inner cube
    auto voids = identify_voids(mesh, closed);
    ASSERT_EQ(voids.size(), 1u);  // one void
    // one void: inner cube
    EXPECT_EQ(voids[0].size(), 12u);  // inner cube has 12 triangles
}

TEST(IdentifyVoids, TwoDisjointClosedCubesReturnsNoVoids) {
    TriangleMesh mesh = make_mesh_from_stl(make_two_disjoint_cubes_stl());
    auto components = find_connected_components(mesh);
    std::vector<ConnectedComponent> closed;
    for (const auto& c : components) {
        if (is_connected_component_closed(mesh, c)) {
            closed.push_back(std::move(c));
        }
    }

    ASSERT_EQ(closed.size(), 2u);  // two closed components
    // two closed components: two disjoint cubes
    auto voids = identify_voids(mesh, closed);
    EXPECT_TRUE(voids.empty());  // no voids -> two disjoint cubes
}

//---------------------------------------------------------------------------
// export_voids_to_stl -> cube, tetrahedron, pyramid shape voids inside cube
//---------------------------------------------------------------------------

static void expect_void_stl_content(const char* filepath) {
    std::ifstream in(filepath);
    ASSERT_TRUE(in) << filepath << " should exist after export";
    std::string s((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    in.close();
    EXPECT_TRUE(s.find("solid") != std::string::npos);
    EXPECT_TRUE(s.find("voids") != std::string::npos);
    EXPECT_TRUE(s.find("vertex") != std::string::npos);
}

TEST(ExportVoidsToStl, WritesCubeVoidToStl) {
    TriangleMesh mesh = make_mesh_from_stl(make_outer_inner_cubes_stl());
    {
        std::ofstream out("void_detection_voids_cube.stl");
        if (out) {
            export_voids_to_stl(mesh, out);
        }
    }
    expect_void_stl_content("void_detection_voids_cube.stl");
}

TEST(ExportVoidsToStl, WritesTetrahedronVoidToStl) {
    TriangleMesh mesh = make_mesh_from_stl(make_outer_cube_inner_tetrahedron_stl());
    {
        std::ofstream out("void_detection_voids_tetrahedron.stl");
        if (out) {
            export_voids_to_stl(mesh, out);
        }
    }
    expect_void_stl_content("void_detection_voids_tetrahedron.stl");
}

TEST(ExportVoidsToStl, WritesPyramidVoidToStl) {
    TriangleMesh mesh = make_mesh_from_stl(make_outer_cube_inner_pyramid_stl());
    {
        std::ofstream out("void_detection_voids_pyramid.stl");
        if (out) {
            export_voids_to_stl(mesh, out);
        }
    }
    expect_void_stl_content("void_detection_voids_pyramid.stl");
}

/// Outer cube [0,0,0]-[4,4,4] with three small closed cubes inside (three voids)
static std::string make_big_cube_with_several_voids_stl() {
    std::string s;
    s += R"(
      solid big_cube_several_voids
    )";
    auto emit_cube = [&s](double x0, double y0, double z0, double x1, double y1, double z1) {
        s += "  facet normal 0 0 -1\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z0) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 0 -1\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " +
             std::to_string(x0) + " " + std::to_string(y1) + " " + std::to_string(z0) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 0 1\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " + std::to_string(x0) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 0 1\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y0) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 -1 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x0) +
             " " + std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y0) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 -1 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y0) + " " + std::to_string(z0) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 1 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 0 1 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x0) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal -1 0 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x0) +
             " " + std::to_string(y1) + " " + std::to_string(z0) + "\n      vertex " +
             std::to_string(x0) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal -1 0 0\n    outer loop\n      vertex " + std::to_string(x0) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x0) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x0) + " " + std::to_string(y0) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 1 0 0\n    outer loop\n      vertex " + std::to_string(x1) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y0) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z1) +
             "\n    endloop\n  endfacet\n";
        s += "  facet normal 1 0 0\n    outer loop\n      vertex " + std::to_string(x1) + " " +
             std::to_string(y0) + " " + std::to_string(z0) + "\n      vertex " + std::to_string(x1) +
             " " + std::to_string(y1) + " " + std::to_string(z1) + "\n      vertex " +
             std::to_string(x1) + " " + std::to_string(y1) + " " + std::to_string(z0) +
             "\n    endloop\n  endfacet\n";
    };
    // Outer cube [0,0,0]-[4,4,4] with three small closed cubes inside (three voids)
    emit_cube(0, 0, 0, 4, 4, 4);
    emit_cube(1, 1, 1, 2, 2, 2);
    emit_cube(2, 2, 0.5, 3, 3, 1.5);
    emit_cube(0.5, 2.5, 2.5, 1.5, 3.5, 3.5);
    s += "endsolid big_cube_several_voids\n";
    return s;
}

//---------------------------------------------------------------------------
// identify_voids -> big cube with several voids
//---------------------------------------------------------------------------

TEST(IdentifyVoids, BigCubeWithSeveralVoidsReturnsAllVoids) {
    TriangleMesh mesh = make_mesh_from_stl(make_big_cube_with_several_voids_stl());
    auto components = find_connected_components(mesh);
    std::vector<ConnectedComponent> closed;
    for (const auto& c : components) {
        if (is_connected_component_closed(mesh, c)) {
            closed.push_back(std::move(c));
        }
    }

    ASSERT_EQ(
        closed.size(), 4u
    ) << "one outer + three inner cubes";  // one outer + three inner cubes
    auto voids = identify_voids(mesh, closed);
    ASSERT_EQ(voids.size(), 3u);      // three voids
    EXPECT_EQ(voids[0].size(), 12u);  // void 1
    EXPECT_EQ(voids[1].size(), 12u);  // void 2
    EXPECT_EQ(voids[2].size(), 12u);  // void 3
}

TEST(ExportVoidsToStl, WritesBigCubeWithSeveralVoidsToStl) {
    TriangleMesh mesh = make_mesh_from_stl(make_big_cube_with_several_voids_stl());
    {
        std::ofstream out("void_detection_voids_big_cube_several.stl");
        if (out) {
            export_voids_to_stl(mesh, out);
        }
    }
    expect_void_stl_content("void_detection_voids_big_cube_several.stl");
}

//---------------------------------------------------------------------------
// detect voids in geometry_with_voids.stl
//---------------------------------------------------------------------------

TEST(VoidDetection, GeometryWithVoids_BinaryStl) {
    std::string binary_path = "geometry_with_voids.stl";
    if (!std::filesystem::exists(binary_path)) {
        // try relative path
        binary_path = "../geometry_with_voids.stl";
    }
    // skip test if file not found
    if (!std::filesystem::exists(binary_path)) {
        GTEST_SKIP() << "geometry_with_voids.stl not found";
    }

    const std::string ascii_path = "geometry_with_voids_ascii.stl";

    auto t0 = std::chrono::steady_clock::now();
    convert_binary_stl_to_ascii(binary_path, ascii_path);
    auto t1 = std::chrono::steady_clock::now();
    TriangleMesh mesh(ascii_path);
    auto t2 = std::chrono::steady_clock::now();
    auto components = find_connected_components(mesh);
    auto t3 = std::chrono::steady_clock::now();

    std::vector<ConnectedComponent> closed;
    for (const auto& c : components) {
        if (is_connected_component_closed(mesh, c)) {
            closed.push_back(c);
        }
    }
    auto t4 = std::chrono::steady_clock::now();
    auto voids = identify_voids(mesh, closed);
    auto t5 = std::chrono::steady_clock::now();

    EXPECT_EQ(voids.size(), 3u);

    {
        std::ofstream out("geometry_with_voids_voids.stl");
        if (out) {
            export_voids_to_stl(mesh, out);
        }
    }
    auto t6 = std::chrono::steady_clock::now();

    using milliseconds = std::chrono::duration<double, std::milli>;
    std::cout << "[VoidDetection] Timing (ms)\n"
              << "  Convert binary STL to ASCII:  "
              << std::chrono::duration_cast<milliseconds>(t1 - t0).count() << " ms\n"
              << "  Build triangle mesh (edge connectivity):          "
              << std::chrono::duration_cast<milliseconds>(t2 - t1).count() << " ms\n"
              << "  Find connected components:     "
              << std::chrono::duration_cast<milliseconds>(t3 - t2).count() << " ms\n"
              << "  Filter closed components:      "
              << std::chrono::duration_cast<milliseconds>(t4 - t3).count() << " ms\n"
              << "  Identify voids:                 "
              << std::chrono::duration_cast<milliseconds>(t5 - t4).count() << " ms\n"
              << "  Export voids to STL:           "
              << std::chrono::duration_cast<milliseconds>(t6 - t5).count() << " ms\n"
              << "  Total:                         "
              << std::chrono::duration_cast<milliseconds>(t6 - t0).count() << " ms\n";
}
