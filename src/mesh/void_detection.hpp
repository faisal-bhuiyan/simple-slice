#pragma once

#include <vector>

#include "geometry.hpp"
#include "triangle_mesh.hpp"

namespace simple_slice::mesh {

//----------------------------------------------------
// Axis aligned bounding box
//----------------------------------------------------

/// Tolerance for floating point comparisons in AABB computations
constexpr double kEpsilon{1e-9};

/**
 * @brief 3D axis-aligned bounding box (AABB)
 */
struct AxisAlignedBoundingBox {
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;

    /**
     * @brief Constructor
     * @param min_x_in The minimum x coordinate
     * @param min_y_in The minimum y coordinate
     * @param min_z_in The minimum z coordinate
     * @param max_x_in The maximum x coordinate
     * @param max_y_in The maximum y coordinate
     * @param max_z_in The maximum z coordinate
     */
    AxisAlignedBoundingBox(
        double min_x_in = 0., double min_y_in = 0., double min_z_in = 0., double max_x_in = 0.,
        double max_y_in = 0., double max_z_in = 0.
    );

    /**
     * @brief Constructor from two points and padding
     * @param a The first point
     * @param b The second point
     * @param pad The padding
     */
    AxisAlignedBoundingBox(const Point& a, const Point& b, double pad = kEpsilon);
};

/**
 * @brief Check if one AABB is contained in another AABB
 * @param outer The outer AABB
 * @param inner The inner AABB
 * @param tol The tolerance for floating point comparisons
 * @return True if the inner AABB is contained in the outer AABB, false otherwise
 */
bool aabb_contains(
    const AxisAlignedBoundingBox&, const AxisAlignedBoundingBox&, double tol = kEpsilon
);

//----------------------------------------------------
// Connected components
//----------------------------------------------------

/// A connected component is a list of triangle indices
using ConnectedComponent = std::vector<TriangleIndex>;

/**
 * @brief Find the connected components in a triangle mesh
 * @param mesh The triangle mesh
 * @return A list of connected components
 */
std::vector<ConnectedComponent> find_connected_components(const TriangleMesh&);
/**
 * @brief Check if a connected component is closed
 * @param mesh The triangle mesh
 * @param component The connected component
 * @return True if the connected component is closed, false otherwise
 */

/**
 * @brief Check if a connected component is closed i.e. all edges are shared by exactly two triangles
 * @param mesh The triangle mesh
 * @param component The connected component
 * @return True if the connected component is closed, false otherwise
 */
bool is_connected_component_closed(const TriangleMesh&, const ConnectedComponent&);

//----------------------------------------------------
// Void detection
//----------------------------------------------------

/**
 * @brief Compute the AABB for a connected component
 * @param mesh The triangle mesh
 * @param component The connected component
 * @param pad The padding
 * @return The AABB for the connected component
 */
AxisAlignedBoundingBox compute_component_aabb(
    const TriangleMesh& mesh, const ConnectedComponent& component, double pad = kEpsilon
);

/**
 * @brief Identify the voids in a triangle mesh
 * @param mesh The triangle mesh
 * @param closed_components The closed connected components
 * @return A list of voids
 */
std::vector<ConnectedComponent> identify_voids(
    const TriangleMesh& mesh, const std::vector<ConnectedComponent>& closed_components
);

/**
 * @brief Export the voids to an STL file
 *
 * This is a wrapper function that finds the connected components, checks if they are closed,
 * identifies the voids, and exports the voids to an STL file.
 *
 * @param mesh The triangle mesh
 * @param out The output stream
 */
void export_voids_to_stl(const TriangleMesh& mesh, std::ostream& out);

}  // namespace simple_slice::mesh
