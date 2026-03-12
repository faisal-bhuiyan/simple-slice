#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

namespace simple_slice::polyline {

/// A 3D point represented by Cartesian coordinates (x, y, z)
using Point = std::array<double, 3>;

/// Integer type used to index vertices in a polyline
using VertexIndex = int32_t;

/// Invalid vertex index used to indicate a vertex is not connected to any other vertex
constexpr VertexIndex kUnconnectedVertex{-1};

/// Classification of a polyline based on its topology
enum class PolylineType {
    kOpen = 0,    ///< Open polyline with distinct start and end vertices
    kClosed = 1,  ///< Closed polyline where start and end vertices are the same
};

/// Representation format used to construct a polyline
enum class PolylineRepresentation {
    kVerboseSegments = 0,           ///< list of vertex index pairs representing individual segments
    kCompressedVertexOrdering = 1,  ///< sequence of vertex indices in traversal order
};

/**
 * @brief Represents a polyline or polygon defined by vertex connectivity.
 *
 * A polyline may be open (two endpoints) or closed (polygon).
 * The class supports construction from either a verbose segment list
 * or a compressed vertex ordering. Internally, the polyline is stored
 * in compressed form for efficient traversal and classification.
 */
class Polyline {
public:
    /**
     * Constructs a polyline from the given representation
     *
     * @param representation Input representation format
     * @param data Vertex indices describing the polyline
     * @param vertices Optional list of vertex coordinates
     *
     * @throws std::invalid_argument if the input data violates polyline validity constraints
     */
    Polyline(
        PolylineRepresentation representation, const std::vector<VertexIndex>& data,
        std::vector<Point> vertices = {}
    );

    /**
     * Returns the list of vertices associated with the polyline
     *
     * @return Reference to the vertex coordinate array
     */
    const std::vector<Point>& GetVertices() const { return vertices_; }

    /**
     * Returns the compressed vertex ordering of the polyline
     *
     * @return Reference to the compressed vertex index sequence
     */
    const std::vector<VertexIndex>& GetCompressedSegments() const { return compressed_segments_; }

    /**
     * Returns the topological type of the polyline
     *
     * @return PolylineType::kOpen or PolylineType::kClosed
     */
    PolylineType GetType() const { return type_; }

    /**
     * Converts a verbose segment representation into a compressed vertex ordering
     *
     * @param segments Flat list of vertex index pairs (2N entries)
     * @param num_vertices Total number of vertices in the index space
     * @return Vertex indices in traversal order
     */
    static std::vector<VertexIndex> GetCompressedVertexOrdering(
        std::span<const VertexIndex> segments, size_t num_vertices
    );

    /**
     * Determines whether the polyline is a closed polygon
     *
     * @return true if the polyline is closed; false otherwise
     */
    bool IsPolygon() const;

private:
    /// List of vertices in the polyline
    std::vector<Point> vertices_;

    /// Compressed vertex ordering representing the polyline traversal
    std::vector<VertexIndex> compressed_segments_;

    /// Topological type of the polyline (open or closed)
    PolylineType type_;
};

}  // namespace simple_slice::polyline
