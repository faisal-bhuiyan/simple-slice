#include "polyline.hpp"

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace simple_slice::polyline {

Polyline::Polyline(
    PolylineRepresentation representation, const std::vector<VertexIndex>& data,
    std::vector<Point> vertices
)
    : vertices_(std::move(vertices)) {
    //----------------------------------------------
    // Checks
    //----------------------------------------------

    // Segment buffer is empty -> throw
    if (data.empty()) {
        throw std::invalid_argument("segments buffer cannot be empty");
    }

    if (representation == PolylineRepresentation::kVerboseSegments) {
        // Number of entries in segment data is NOT even -> throw
        if (data.size() % 2 != 0) {
            throw std::invalid_argument("segments buffer must contain an even number of entries");
        }

        /**
         * NOTE: This validation is not optimal for performance, but it establishes the assumptions
         * made into the class invariants i.e. single-connected polyline/polygon and degree is 2 (1
         * for endpoints). I am prioritizing robustness over performance here. The hot path in
         * GetCompressedVertexOrdering() assumes valid input.
         */

        // Find the vertex with largest index in the segments buffer to determine vertex count
        const size_t num_segments{data.size() / 2};

        VertexIndex max_vertex{0};
        for (const auto vertex : data) {
            max_vertex = std::max(max_vertex, vertex);
        }

        // Count degree of each vertex i.e. number of segments it participates in
        std::vector<uint8_t> degree_of_vertices(static_cast<size_t>(max_vertex) + 1, 0);
        for (size_t i = 0; i < num_segments; ++i) {
            // Each segment connects two vertices -> increment degree counts
            ++degree_of_vertices[static_cast<size_t>(data[2 * i])];
            ++degree_of_vertices[static_cast<size_t>(data[2 * i + 1])];
        }

        // Validate single-connected polyline/polygon assumptions:
        // - every vertex has degree 1 or 2
        // - exactly 0 degree-1 vertices (polygon) or exactly 2 vertices (open polyline)
        uint8_t degree_1_count{0};
        for (VertexIndex v = 0; v <= max_vertex; ++v) {
            // Ignore unconnected vertices (degree 0)
            if (degree_of_vertices[static_cast<size_t>(v)] == 0) {
                continue;
            }

            // Count degree-1 endpoints
            if (degree_of_vertices[static_cast<size_t>(v)] == 1) {
                ++degree_1_count;
            }

            // Validate degree <= 2 for single-connected polyline/polygon
            if (degree_of_vertices[static_cast<size_t>(v)] > 2) {
                throw std::invalid_argument(
                    "vertex " + std::to_string(v) + " has degree " +
                    std::to_string(degree_of_vertices[static_cast<size_t>(v)]) +
                    "; expected at most 2 for a single connected polyline/polygon"
                );
            }
        }

        // We have a problem if number of degree-1 endpoints is NOT 0 or 2 -> throw
        if (degree_1_count != 0 && degree_1_count != 2) {
            throw std::invalid_argument(
                "expected 0 or 2 degree-1 endpoints, found " + std::to_string(degree_1_count)
            );
        }

        // Now that we have validated the input, we can build the compressed vertex ordering
        // and use it for storing the segments with optimal memory footprint
        this->compressed_segments_ =
            GetCompressedVertexOrdering(data, static_cast<size_t>(max_vertex) + 1);

    } else if (representation == PolylineRepresentation::kCompressedVertexOrdering) {
        // Data is already in compressed vertex ordering form -> store directly
        this->compressed_segments_ = data;
    } else {
        // Invalid representation value -> throw
        throw std::invalid_argument("Invalid PolylineRepresentation value: not supported");
    }

    // Determine polyline type based on compressed vertex ordering
    this->type_ = IsPolygon() ? PolylineType::kClosed : PolylineType::kOpen;
}

std::vector<VertexIndex> Polyline::GetCompressedVertexOrdering(
    std::span<const VertexIndex> segments, size_t num_vertices
) {
    //----------------------------------------------
    // Build vertex connectivity
    //----------------------------------------------

    const size_t num_segments{segments.size() / 2};

    // Each vertex connects to up to two others (-1 = unconnected vertex)
    std::vector<std::pair<VertexIndex, VertexIndex>> vertex_connectivity(
        num_vertices, {kUnconnectedVertex, kUnconnectedVertex}
    );

    // Assign a neighbor to the first available slot (-1) of a vertex
    auto assign_neighbor = [&vertex_connectivity](VertexIndex vertex, VertexIndex neighbor) {
        if (vertex_connectivity[static_cast<size_t>(vertex)].first == kUnconnectedVertex) {
            vertex_connectivity[static_cast<size_t>(vertex)].first = neighbor;
            return;
        }
        // first slot is already taken -> use second slot
        vertex_connectivity[static_cast<size_t>(vertex)].second = neighbor;
    };

    // Each segment connects two vertices -> build connectivity pairs by looping over segments
    for (size_t i_segment = 0; i_segment < num_segments; ++i_segment) {
        const VertexIndex vertex_1 = segments[2 * i_segment];
        const VertexIndex vertex_2 = segments[2 * i_segment + 1];
        assign_neighbor(vertex_1, vertex_2);  // vertex_2 -- neighbor --> vertex_1
        assign_neighbor(vertex_2, vertex_1);  // vertex_1 -- neighbor --> vertex_2
    }

    //----------------------------------------------
    // Determine polyline type
    //----------------------------------------------

    // Find endpoints (degree-1 vertices: only .first is set AND .second is -1)
    std::vector<VertexIndex> endpoints;
    for (size_t vertex = 0; vertex < num_vertices; ++vertex) {
        const VertexIndex v = static_cast<VertexIndex>(vertex);
        if (vertex_connectivity[static_cast<size_t>(v)].first != kUnconnectedVertex &&
            vertex_connectivity[static_cast<size_t>(v)].second == kUnconnectedVertex) {
            endpoints.push_back(v);
        }
    }

    // Two scenarios possible:
    // Polyline: 2 endpoints (2 vertices w/ degree 1, rest degree 2)
    // Polygon: no endpoints (all vertices w/ degree 2)
    const bool is_closed{endpoints.empty()};
    VertexIndex starting_vertex{0};
    if (is_closed) {
        // POLYGON -> start at the smallest participating vertex (no determinism requirement here)
        for (size_t vertex = 0; vertex < num_vertices; ++vertex) {
            const VertexIndex v = static_cast<VertexIndex>(vertex);
            if (vertex_connectivity[static_cast<size_t>(v)].first != kUnconnectedVertex) {
                starting_vertex = v;
                break;
            }
        }
    } else {
        // POLYLINE -> start at the smaller endpoint of the two (to satisfy DETERMINISM)
        starting_vertex = std::min(endpoints[0], endpoints[1]);
    }

    //----------------------------------------------
    // Build compressed vertex ordering
    //----------------------------------------------

    // Walk the chain: at each step, advance to the next neighbor that is not the previous vertex
    std::vector<VertexIndex> compressed_ordering;
    compressed_ordering.reserve(num_vertices + (is_closed ? 1 : 0));
    compressed_ordering.push_back(starting_vertex);

    // Lambda: given a vertex, return the neighbor that is not `already_visited`
    // Returns -1 if no such neighbor exists (reached the end of open polyline)
    auto next_neighbor =
        [&vertex_connectivity](VertexIndex vertex, VertexIndex already_visited) -> VertexIndex {
        const auto [v1, v2] = vertex_connectivity[static_cast<size_t>(vertex)];
        if (v1 != kUnconnectedVertex && v1 != already_visited) {
            return v1;
        }
        if (v2 != kUnconnectedVertex && v2 != already_visited) {
            return v2;
        }
        // No unvisited neighbors -> end of open polyline
        return kUnconnectedVertex;
    };

    // First step: just take the first neighbor (determinism for polylines is already guaranteed by
    // starting at the smaller endpoint for open polylines)
    VertexIndex previous_vertex{starting_vertex};
    VertexIndex current_vertex{vertex_connectivity[static_cast<size_t>(starting_vertex)].first};

    compressed_ordering.push_back(current_vertex);

    // Subsequent steps: walk the chain until we reach the end of an open polyline or loop back to
    // the start of a polygon
    while (true) {
        VertexIndex next_vertex = next_neighbor(current_vertex, previous_vertex);

        // Break condition for polyline: reached the other endpoint
        if (next_vertex == kUnconnectedVertex) {
            break;
        }

        compressed_ordering.push_back(next_vertex);

        // Break condition for polygon: completed the loop back to the starting vertex
        if (is_closed && next_vertex == starting_vertex) {
            break;
        }

        // Advance the walk: update previous and current vertices
        previous_vertex = current_vertex;
        current_vertex = next_vertex;
    }

    return compressed_ordering;
}

bool Polyline::IsPolygon() const {
    // A polygon's compressed ordering starts and ends with the same vertex
    return compressed_segments_.size() >= 2 &&
           compressed_segments_.front() == compressed_segments_.back();
}

}  // namespace simple_slice::polyline
