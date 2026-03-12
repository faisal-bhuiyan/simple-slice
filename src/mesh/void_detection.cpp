#include "void_detection.hpp"

#include <array>
#include <queue>
#include <vector>

#include "geometry.hpp"
#include "stl_io.hpp"

namespace simple_slice::mesh {

AxisAlignedBoundingBox::AxisAlignedBoundingBox(
    double min_x_in, double min_y_in, double min_z_in, double max_x_in, double max_y_in,
    double max_z_in
)
    : min_x(min_x_in),
      min_y(min_y_in),
      min_z(min_z_in),
      max_x(max_x_in),
      max_y(max_y_in),
      max_z(max_z_in) {
    if (min_x > max_x || min_y > max_y || min_z > max_z) {
        throw std::invalid_argument("AxisAlignedBoundingBox: min must be <= max on all axes");
    }
}

AxisAlignedBoundingBox::AxisAlignedBoundingBox(const Point& a, const Point& b, double pad)
    : AxisAlignedBoundingBox(
          std::min(a[0], b[0]) - pad, std::min(a[1], b[1]) - pad, std::min(a[2], b[2]) - pad,
          std::max(a[0], b[0]) + pad, std::max(a[1], b[1]) + pad, std::max(a[2], b[2]) + pad
      ) {
}

bool aabb_contains(
    const AxisAlignedBoundingBox& outer, const AxisAlignedBoundingBox& inner, double tol
) {
    return outer.min_x <= inner.min_x - tol && inner.max_x + tol <= outer.max_x &&
           outer.min_y <= inner.min_y - tol && inner.max_y + tol <= outer.max_y &&
           outer.min_z <= inner.min_z - tol && inner.max_z + tol <= outer.max_z;
}

std::vector<ConnectedComponent> find_connected_components(const TriangleMesh& mesh) {
    const auto& triangles{mesh.GetTriangles()};
    const auto& edge_connectivity{mesh.GetEdgeConnectivity()};
    const std::size_t num_triangles{triangles.size()};

    std::vector<bool> visited(num_triangles, false);  // list of visited triangles
    std::vector<ConnectedComponent> components;       // list of connected components
    components.reserve(20);  // estimating the number of components as a heuristic

    // Lambda: find the smallest unvisited triangle index, or return num_triangles if none
    const auto smallest_unvisited_triangle_index = [&visited, num_triangles]() -> std::size_t {
        for (std::size_t i = 0; i < num_triangles; ++i) {
            if (!visited[i]) {
                return i;  // return the smallest unvisited triangle index
            }
        }
        return num_triangles;  // return num_triangles if no unvisited triangles
    };

    // Repeatedly pick an unvisited triangle as seed -> perform BFS to get one connected component
    while (true) {
        const std::size_t seed{smallest_unvisited_triangle_index()};
        if (seed >= num_triangles) {
            break;  // no unvisited triangles -> stop
        }

        ConnectedComponent component;
        std::queue<std::size_t> queue;
        visited[seed] = true;
        queue.push(seed);

        // Propagate the connected component through the mesh using BFS
        while (!queue.empty()) {
            // Get the next triangle to visit from the FIFO queue
            const std::size_t triangle_index{queue.front()};
            queue.pop();
            component.push_back(static_cast<TriangleIndex>(triangle_index));

            const Triangle& triangle{triangles[triangle_index]};
            // Examine all three edges of this triangle
            const std::array<Edge, 3> edges{
                make_edge(triangle.a, triangle.b),  // edge 1
                make_edge(triangle.b, triangle.c),  // edge 2
                make_edge(triangle.c, triangle.a)   // edge 3
            };

            // For each edge of triangle: find neighbor triangle and check if it is already visited
            for (const Edge& edge : edges) {
                // Find the neighbor triangle for this edge
                auto it{edge_connectivity.find(edge)};
                if (it == edge_connectivity.end()) {
                    continue;  // edge not found in the mesh -> skip
                }

                // Check if edge is boundary or non-manifold -> skip
                const auto& degree_of_edge{it->second};
                if (degree_of_edge[1] == kBoundaryTriangleIndex) {
                    continue;  // boundary edge -> skip
                }

                // Get neighbor triangle index from degree of edge
                // Logic: Only 2 triangles share an edge -> one we are on OR neighbor triangle
                const std::size_t neighbor_index{
                    (degree_of_edge[0] == static_cast<TriangleIndex>(triangle_index))
                        ? static_cast<std::size_t>(degree_of_edge[1])
                        : static_cast<std::size_t>(degree_of_edge[0])
                };

                // If neighbor triangle is already visited -> skip
                if (visited[neighbor_index]) {
                    continue;
                }

                // Mark neighbor as visited and add it to the FIFO queue
                visited[neighbor_index] = true;
                queue.push(neighbor_index);
            }
        }

        // Add the connected component to the list of components
        components.push_back(std::move(component));
    }

    return components;
}

bool is_connected_component_closed(const TriangleMesh& mesh, const ConnectedComponent& component) {
    const auto& triangles{mesh.GetTriangles()};
    const auto& edge_connectivity{mesh.GetEdgeConnectivity()};

    // Check if all edges of the component are shared by exactly two triangles
    for (const TriangleIndex& triangle_index : component) {
        const Triangle& triangle{triangles[static_cast<std::size_t>(triangle_index)]};

        // Examine all three edges of this triangle
        const std::array<Edge, 3> edges{
            make_edge(triangle.a, triangle.b),  // edge 1
            make_edge(triangle.b, triangle.c),  // edge 2
            make_edge(triangle.c, triangle.a)   // edge 3
        };

        // For each edge of triangle: find neighbor triangle and check if it is shared by exactly two
        // triangles
        for (const Edge& edge : edges) {
            auto it{edge_connectivity.find(edge)};
            if (it == edge_connectivity.end()) {
                // Should not happen if invariants hold -> treat as open
                return false;
            }

            const auto& degree_of_edge{it->second};
            // Boundary edge -> component is open
            if (degree_of_edge[1] == kBoundaryTriangleIndex) {
                return false;
            }
        }
    }
    // If we are here, all edges are shared by exactly two triangles -> component is closed
    return true;
}

AxisAlignedBoundingBox compute_component_aabb(
    const TriangleMesh& mesh, const ConnectedComponent& component, double pad
) {
    const auto& triangles{mesh.GetTriangles()};

    // Initialize AABB from the first triangle's three vertices (min/max over a, b, c)
    const Triangle& t0{triangles[static_cast<std::size_t>(component[0])]};
    AxisAlignedBoundingBox box{std::min(t0.a[0], std::min(t0.b[0], t0.c[0])),
                               std::min(t0.a[1], std::min(t0.b[1], t0.c[1])),
                               std::min(t0.a[2], std::min(t0.b[2], t0.c[2])),
                               std::max(t0.a[0], std::max(t0.b[0], t0.c[0])),
                               std::max(t0.a[1], std::max(t0.b[1], t0.c[1])),
                               std::max(t0.a[2], std::max(t0.b[2], t0.c[2]))};

    // Lambda: expand the AABB to include provided point
    auto expand = [&](const Point& point) {
        box.min_x = std::min(box.min_x, point[0]);
        box.min_y = std::min(box.min_y, point[1]);
        box.min_z = std::min(box.min_z, point[2]);
        box.max_x = std::max(box.max_x, point[0]);
        box.max_y = std::max(box.max_y, point[1]);
        box.max_z = std::max(box.max_z, point[2]);
    };

    // Expand the AABB to include all triangles in the component
    for (const TriangleIndex& index : component) {
        const Triangle& t{triangles[static_cast<std::size_t>(index)]};
        expand(t.a);
        expand(t.b);
        expand(t.c);
    }

    // Add padding to the AABB
    box.min_x -= pad;
    box.min_y -= pad;
    box.min_z -= pad;
    box.max_x += pad;
    box.max_y += pad;
    box.max_z += pad;

    return box;
}

std::vector<ConnectedComponent> identify_voids(
    const TriangleMesh& mesh, const std::vector<ConnectedComponent>& closed_components
) {
    if (closed_components.size() < 2U) {
        return {};  // 0 or 1 closed component -> no voids
    }

    // Compute the AABB for each closed component
    std::vector<AxisAlignedBoundingBox> component_aabbs;
    component_aabbs.reserve(closed_components.size());
    for (const ConnectedComponent& component : closed_components) {
        component_aabbs.push_back(compute_component_aabb(mesh, component));
    }

    // A component is a void if its AABB is contained in the AABB of any other component
    std::vector<ConnectedComponent> voids{};
    voids.reserve(closed_components.size());
    for (std::size_t i = 0; i < closed_components.size(); ++i) {
        const AxisAlignedBoundingBox& inner = component_aabbs[i];
        bool is_void{false};
        for (std::size_t j = 0; j < closed_components.size(); ++j) {
            if (i == j) {
                continue;  // skip self comparison
            }
            // Check if the AABB of the inner component is contained in the AABB of the outer
            // component
            if (aabb_contains(component_aabbs[j], inner)) {
                is_void = true;
                break;  // inner component is a void -> no need to check other components
            }
        }
        if (is_void) {
            voids.push_back(closed_components[i]);
        }
    }
    return voids;
}

void export_voids_to_stl(const TriangleMesh& mesh, std::ostream& out) {
    // step 1: find the connected components in triangle mesh
    std::vector<ConnectedComponent> connected_components{find_connected_components(mesh)};

    // Step 2: look for the closed connected components out of all connected components
    std::vector<ConnectedComponent> closed_components{};
    closed_components.reserve(connected_components.size());
    for (const ConnectedComponent& component : connected_components) {
        if (is_connected_component_closed(mesh, component)) {
            closed_components.push_back(std::move(component));
        }
    }

    // step 3: identify the voids out of all closed connected components
    std::vector<ConnectedComponent> voids{identify_voids(mesh, closed_components)};

    // step 4: flatten void components to triangles and write in ASCII STL format to output stream
    std::vector<Triangle> triangles_to_write;
    const auto& all_triangles = mesh.GetTriangles();
    for (const ConnectedComponent& comp : voids) {
        for (TriangleIndex idx : comp) {
            triangles_to_write.push_back(all_triangles[static_cast<std::size_t>(idx)]);
        }
    }
    write_ascii_stl(out, "voids", triangles_to_write);
}

}  // namespace simple_slice::mesh
