#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry.hpp"

namespace simple_slice::mesh {

/// Index type for triangles in the mesh (max ~2 billion triangles)
using TriangleIndex = std::int32_t;

/// Placeholder for boundary triangles (only one triangle shares the edge)
constexpr TriangleIndex kBoundaryTriangleIndex{static_cast<TriangleIndex>(-1)};

/// Tolerance for floating point comparisons
constexpr double kTolerance{1e-16};

/**
 * @brief Triangle mesh loaded from an ASCII STL file
 *
 * The mesh stores a collection of triangles and provides functionality to build edge-to-triangle
 * connectivity information. Edges are treated in canonical form to ensure consistent
 * adjacency/connectivity mapping.
 */
class TriangleMesh {
public:
    /**
     * @brief Constructs an empty triangle mesh
     */
    TriangleMesh() = default;

    /**
     * @brief Constructs a mesh by parsing an ASCII STL file
     *
     * The file at the provided path is parsed and all triangles contained in the STL are loaded into
     * the mesh.
     *
     * @param path Path to an ASCII STL file
     *
     * @throws std::runtime_error if the file cannot be opened or parsed
     *
     * @note Constructor is explicit to avoid implicit conversion from path strings to mesh;
     *       constructing a mesh does I/O and parsing, so call sites should be explicit.
     */
    explicit TriangleMesh(const std::string& path);

    /**
     * @brief Builds the EDGE -> TRIANGLE connectivity map
     *
     * For each triangle in the mesh, its three edges are inserted into a connectivity map that
     * associates each canonical edge with the indices of triangles that share it. A manifold mesh
     * is guaranteed to have a maximum of 2 triangles sharing each edge.
     */
    void BuildEdgeToTriangleConnectivity();

    /**
     * @brief Returns the list of triangles in the mesh
     *
     * @return Reference to the triangle container
     */
    const std::vector<Triangle>& GetTriangles() const { return triangles_; }

    /**
     * @brief Returns the edge-to-triangle connectivity map
     *
     * Each entry maps a canonical edge to the indices of triangles that share that edge.
     *
     * @return Reference to the edge connectivity map
     */
    const std::unordered_map<Edge, std::array<TriangleIndex, 2>, EdgeHash, EdgeEquality>&
    GetEdgeConnectivity() const {
        return edge_connectivity_;
    }

private:
    /// List of triangles in the mesh
    std::vector<Triangle> triangles_;

    /// Maps each canonical edge to the indices of triangles that share it
    std::unordered_map<Edge, std::array<TriangleIndex, 2>, EdgeHash, EdgeEquality>
        edge_connectivity_;
};

}  // namespace simple_slice::mesh
