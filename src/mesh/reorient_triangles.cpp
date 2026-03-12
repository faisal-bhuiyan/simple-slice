#include "reorient_triangles.hpp"

#include <array>
#include <queue>
#include <vector>

#include "geometry.hpp"
#include "stl_io.hpp"

namespace simple_slice::mesh {

void flip_triangle(Triangle& triangle) {
    std::swap(triangle.b, triangle.c);
}

bool has_directed_edge(const Triangle& triangle, const Point& from, const Point& to) {
    // Check if provided edge is in the triangle in correct order
    return (triangle.a == from && triangle.b == to) || (triangle.b == from && triangle.c == to) ||
           (triangle.c == from && triangle.a == to);
}

bool are_orientations_consistent(const Triangle& t1, const Triangle& t2, const Edge& edge) {
    bool t1_edge{has_directed_edge(t1, edge.first, edge.second)};
    bool t2_edge{has_directed_edge(t2, edge.first, edge.second)};

    // orientations are consistent if edges are in OPPOSITE order
    return t1_edge != t2_edge;
}

std::vector<Triangle> reorient_inconsistent_triangles(TriangleMesh& mesh, std::size_t seed) {
    const auto& triangles{mesh.GetTriangles()};
    const auto& edge_connectivity{mesh.GetEdgeConnectivity()};

    if (seed >= triangles.size()) {
        // seed is out of range -> no triangles to reorient
        return {};
    }

    std::vector<bool> visited_triangles(triangles.size(), false);  // list of visited triangles
    std::vector<Triangle> flipped_triangles;                       // list of flipped triangles
    std::queue<std::size_t> queue;  // queue of triangles to visit (FIFO)

    visited_triangles[seed] = true;
    queue.push(seed);

    // Propagate the orientation through the mesh using BFS
    while (!queue.empty()) {
        std::size_t triangle_index{queue.front()};
        queue.pop();

        const Triangle& triangle{triangles[triangle_index]};

        // Examine all three edges of this triangle
        std::array<Edge, 3> edges{
            make_edge(triangle.a, triangle.b), make_edge(triangle.b, triangle.c),
            make_edge(triangle.c, triangle.a)
        };

        // For each edge of triangle: find neighbor triangle and check if orientations are consistent
        for (const Edge& edge : edges) {
            // Find the neighbor triangle for this edge
            auto it{edge_connectivity.find(edge)};
            if (it == edge_connectivity.end()) {
                continue;  // edge not found in the mesh
            }

            // Check if edge is boundary or non-manifold -> skip
            const auto& degree_of_edge{it->second};
            if (degree_of_edge[1] == kBoundaryTriangleIndex) {
                continue;  // boundary edge -> skip
            }

            // Get neighbor triangle index from degree of edge
            std::size_t neighbor_index{
                (degree_of_edge[0] == static_cast<TriangleIndex>(triangle_index))
                    ? static_cast<std::size_t>(degree_of_edge[1])
                    : static_cast<std::size_t>(degree_of_edge[0])
            };

            // If neighbor triangle is already visited -> skip
            if (visited_triangles[neighbor_index]) {
                continue;
            }

            // If orientations are inconsistent -> flip neighbor triangle orientation
            if (!are_orientations_consistent(triangle, triangles[neighbor_index], edge)) {
                // copy the triangle to be flipped to avoid modifying the original triangle
                Triangle to_be_flipped{triangles[neighbor_index]};
                flip_triangle(to_be_flipped);
                flipped_triangles.push_back(to_be_flipped);
            }

            // Mark neighbor as visited and add it to the queue
            visited_triangles[neighbor_index] = true;
            queue.push(neighbor_index);
        }
    }
    return flipped_triangles;
}

void export_inconsistent_triangles(TriangleMesh& mesh, std::size_t seed, std::ostream& out) {
    // step 1: reorient the inconsistent triangles
    std::vector<Triangle> flipped_triangles{reorient_inconsistent_triangles(mesh, seed)};
    // step 2: write the reoriented triangles to the output stream
    write_ascii_stl(out, "reoriented_triangles", flipped_triangles);
}

}  // namespace simple_slice::mesh
