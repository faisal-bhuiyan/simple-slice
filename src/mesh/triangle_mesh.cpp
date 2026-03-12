#include "triangle_mesh.hpp"

#include <fstream>
#include <stdexcept>

#include "stl_io.hpp"

namespace simple_slice::mesh {

TriangleMesh::TriangleMesh(const std::string& path) {
    //----------------------------------------------
    // Checks
    //----------------------------------------------

    std::ifstream file(path);
    if (!file) {
        throw std::invalid_argument("failed to open STL file: " + path);
    }
    this->triangles_ = parse_ascii_stl(file);

    // Empty mesh -> throw
    if (this->triangles_.empty()) {
        throw std::invalid_argument("triangle mesh cannot be empty");
    }

    // Check for degenerate triangles (two vertices are the same or all three vertices lie on the
    // same line) -> throw
    for (std::size_t i = 0; i < this->triangles_.size(); ++i) {
        const auto& t = this->triangles_[i];

        // Check if any two vertices are the same -> throw
        if (t.a == t.b || t.b == t.c || t.c == t.a) {
            throw std::invalid_argument(
                "degenerate triangle at index " + std::to_string(i) + ": duplicate vertices"
            );
        }

        // More comprehensive and robust check than above:
        // Check if vertices are lies on the same line i.e. area of triangle is zero
        // Area = 0.5 * |(b-a) × (c-a)|
        double v1[3] = {t.b[0] - t.a[0], t.b[1] - t.a[1], t.b[2] - t.a[2]};
        double v2[3] = {t.c[0] - t.a[0], t.c[1] - t.a[1], t.c[2] - t.a[2]};
        double cross_x = v1[1] * v2[2] - v1[2] * v2[1];
        double cross_y = v1[2] * v2[0] - v1[0] * v2[2];
        double cross_z = v1[0] * v2[1] - v1[1] * v2[0];
        double area_squared = cross_x * cross_x + cross_y * cross_y + cross_z * cross_z;
        if (area_squared < kTolerance * kTolerance) {
            throw std::invalid_argument(
                "degenerate triangle at index " + std::to_string(i) +
                ": vertices are co-linear (area is effectively zero)"
            );
        }
    }

    // Build connectivity and validate manifold assumptions
    this->BuildEdgeToTriangleConnectivity();
}

void TriangleMesh::BuildEdgeToTriangleConnectivity() {
    this->edge_connectivity_.clear();
    this->edge_connectivity_.reserve(3 * this->triangles_.size());

    // For each triangle, add its 3 edges to the edge-to-triangle connectivity map
    for (std::size_t i = 0; i < this->triangles_.size(); ++i) {
        const auto& triangle = this->triangles_[i];
        const std::array<Edge, 3> edges{
            make_edge(triangle.a, triangle.b),  // edge 1
            make_edge(triangle.b, triangle.c),  // edge 2
            make_edge(triangle.c, triangle.a)   // edge 3
        };

        // For each edge, add the triangle index to the edge-to-triangle connectivity map
        for (const Edge& edge : edges) {
            // Try to insert the edge into the edge-to-triangle connectivity map
            // should work for the first time
            auto [it, inserted] = this->edge_connectivity_.try_emplace(
                edge,  // edge to be inserted into unordered map
                std::array<TriangleIndex, 2>{
                    static_cast<TriangleIndex>(i),  // index 1: current triangle index
                    kBoundaryTriangleIndex          // index 2: boundary triangle index
                }
            );

            // above insertion failed -> edge is already in the connectivity map
            // we need to check if the edge is shared by 3 or more triangles
            if (!inserted) {
                // Check for NON-MANIFOLD edges (shared by 3 or more triangles) -> throw
                // Logic: if the second connection slot is NOT the boundary triangle index (i.e. we
                // cannot insert the current triangle index into the second position), then the edge
                // must be shared by 3 or more triangles
                if (it->second[1] != kBoundaryTriangleIndex) {
                    throw std::invalid_argument(
                        "non-manifold mesh detected: edge shared by more than 2 triangles"
                    );
                }
                // Edge is already in the connectivity map -> add current triangle index to second
                // position -- at this point, the edge degree is 2 i.e. shared by 2 triangles -- next
                // insertion attempt will throw
                it->second[1] = static_cast<TriangleIndex>(i);
            }
        }
    }
}

}  // namespace simple_slice::mesh
