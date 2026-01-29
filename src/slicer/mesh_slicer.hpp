#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include "geometry/utilities.hpp"
#include "slicer/shapes.hpp"

namespace simple_slice::slicer {

//----------------------------------------------------
// Helpers
//----------------------------------------------------

/**
 * @brief 2D line segment in the XY plane
 */
struct Segment2D {
    Point start_point;
    Point end_point;
};

/**
 * @brief Check if two points are close in XY
 *
 * Intent: robust endpoint matching while stitching segments.
 */
inline bool points_close_2d(const Point& point1, const Point& point2, double epsilon) {
    return std::abs(point1.GetX() - point2.GetX()) <= epsilon &&
           std::abs(point1.GetY() - point2.GetY()) <= epsilon;
}

/**
 * @brief Add point if not already present (XY comparison)
 *
 * Intent: prevent duplicate intersections due to shared vertices or numerical noise.
 */
inline void add_unique_point(std::vector<Point>& points, const Point& point, double epsilon) {
    for (const auto& existing_point : points) {
        if (points_close_2d(existing_point, point, epsilon)) {
            return;  // point already exists
        }
    }
    points.push_back(point);
}

//----------------------------------------------------
// Mesh slicing
//----------------------------------------------------

/**
 * @brief Intersect a triangle with a horizontal plane Z = z
 *
 * Goal: compute the line segment (if any) where a triangle crosses the plane.
 *
 * Algorithm:
 *  1. Iterate the three edges of the triangle (a->b, b->c, c->a)
 *  2. For each edge, compute signed distances to the plane for the two endpoints:
    (d0 = p0.z - plane_height and d1 = p1.z - plane_height)
 *  3. If both endpoints are on the plane, skip the edge (coplanar case)
 *  4. If one endpoint is on the plane, record that vertex
 *  5. If the edge crosses the plane (d0 * d1 < 0), linearly interpolate the XY intersection
 *  6. If exactly two intersection points are found, return them as a line segment. Otherwise,
 *     return false (no valid segment).
 *
 * Assumptions:
 *  - Coplanar triangles/edges are ignored to avoid ambiguous overlapping segments.
 *  - At most one segment is emitted per triangle per layer.
 *
 * Note:
 * This function extracts at most one line segment per triangle per Z layer. It intentionally
 * ignores fully coplanar edges/triangles to keep slicing simple.
 *
 * @param triangle Triangle to intersect
 * @param plane_height Plane height
 * @param output_segment Output segment (valid only if function returns true)
 * @param epsilon Numerical tolerance for classifying points on the plane
 * @return true if a single segment was found, false otherwise
 */
inline bool triangle_plane_segment(
    const Triangle& triangle, double plane_height, Segment2D& output_segment, double epsilon
) {
    std::vector<Point> intersections{};
    intersections.reserve(2);  // maximum number of intersections

    const Point vertices[3] = {triangle.a, triangle.b, triangle.c};
    for (std::size_t i = 0; i < 3; ++i) {
        // Get the vertices of the triangle
        const Point& p0 = vertices[i];
        const Point& p1 = vertices[(i + 1) % 3];

        // Compute the distance from the vertices to the plane
        const double d0 = p0.GetZ() - plane_height;
        const double d1 = p1.GetZ() - plane_height;

        // Edge lies in the plane -> skip to avoid producing ambiguous segments
        if (std::abs(d0) <= epsilon && std::abs(d1) <= epsilon) {
            continue;
        }

        // Vertex lies on the plane -> add it as an intersection
        if (std::abs(d0) <= epsilon) {
            add_unique_point(intersections, Point{p0.GetX(), p0.GetY(), 0.}, epsilon);
            continue;
        }

        // Vertex lies on the plane -> add it as an intersection
        if (std::abs(d1) <= epsilon) {
            add_unique_point(intersections, Point{p1.GetX(), p1.GetY(), 0.}, epsilon);
            continue;
        }

        // Edge crosses the plane -> compute intersection by linear interpolation
        if (d0 * d1 < 0.) {
            const double t = d0 / (d0 - d1);  // interpolation parameter
            const double x = p0.GetX() + t * (p1.GetX() - p0.GetX());
            const double y = p0.GetY() + t * (p1.GetY() - p0.GetY());
            add_unique_point(intersections, Point{x, y, 0.}, epsilon);
        }
    }

    // If not exactly two intersections were found, return false
    if (intersections.size() != 2) {
        return false;
    }

    // Set the output segment to the two intersections
    output_segment.start_point = intersections[0];
    output_segment.end_point = intersections[1];
    return true;
}

/**
 * @brief Stitch unordered segments into polylines by matching endpoints
 *
 * Goal: turn a set of unordered line segments into ordered polylines by
 * connecting endpoints that are “close enough” in XY.
 *
 * Algorithm:
 *  1. Pop one segment and start a new path with its two endpoints.
 *  2. Repeatedly scan remaining segments and attach any that match either
 *     end of the current path (prepend/append as needed).
 *  3. After attaching a segment, remove it and restart the scan.
 *  4. If the path endpoints are close, close the loop by repeating the start point.
 *  5. Repeat until all segments are consumed.
 *
 * Notes:
 *  - This is a greedy endpoint-matching stitcher.
 *  - It is O(n^2) in the number of segments.
 *  - It works well for clean contours but may struggle with ambiguous or noisy segments.
 *
 * @param line_segments Unordered XY segments
 * @param epsilon Endpoint matching tolerance
 * @return Paths assembled from segments (closed when possible)
 */
inline std::vector<Path> stitch_segments_into_paths(
    std::vector<Segment2D> line_segments, double epsilon
) {
    std::vector<Path> polylines{};
    polylines.reserve(line_segments.size());

    while (!line_segments.empty()) {
        // Start a new path from an arbitrary segment
        Segment2D seed_segment = line_segments.back();
        line_segments.pop_back();

        Path path{};
        path.push_back(seed_segment.start_point);
        path.push_back(seed_segment.end_point);

        bool found_match{true};
        while (found_match) {
            found_match = false;
            for (std::size_t i = 0; i < line_segments.size(); ++i) {
                const Segment2D& candidate_segment = line_segments[i];
                // If a segment’s start matches the path’s back, append its end
                if (points_close_2d(path.back(), candidate_segment.start_point, epsilon)) {
                    path.push_back(candidate_segment.end_point);
                }
                // If a segment’s end matches the path’s back, append its start
                else if (points_close_2d(path.back(), candidate_segment.end_point, epsilon)) {
                    path.push_back(candidate_segment.start_point);
                }
                // If a segment’s start matches the path’s front, prepend its end
                else if (points_close_2d(path.front(), candidate_segment.start_point, epsilon)) {
                    path.insert(path.begin(), candidate_segment.end_point);
                }
                // If a segment’s end matches the path’s front, prepend its start
                else if (points_close_2d(path.front(), candidate_segment.end_point, epsilon)) {
                    path.insert(path.begin(), candidate_segment.start_point);
                } else {
                    continue;
                }
                line_segments.erase(line_segments.begin() + i);
                found_match = true;
                break;
            }
        }

        // Close the loop if the endpoints are nearly coincident
        if (path.size() > 2 && points_close_2d(path.front(), path.back(), epsilon) == true) {
            path.push_back(path.front());
        }
        polylines.push_back(std::move(path));
    }
    return polylines;
}

/**
 * @brief Slice a triangle mesh into horizontal layers
 *
 * Goal: slice a triangle mesh into horizontal layers at fixed Z intervals
 * and return 2D polylines per layer.
 *
 * Algorithm:
 *  1. Early returns: if mesh is empty or layer_height <= 0, return empty layers.
 *  2. Compute Z bounds: find min_z and max_z across all triangle vertices
 *     to define the vertical extent of the mesh.
 *  3. Calculate layer count: floor(height / layer_height + 1.0 + 1e-12)
 *     to include both min_z and max_z layers (1e-12 avoids rounding issues).
 *  4. For each layer:
 *     - Compute Z height: z = min_z + i * layer_height
 *     - Intersect all triangles with plane Z = z to get segments
 *     - Stitch segments into polylines using stitch_segments_into_paths
 *     - Store the layer as {z, paths}
 *
 * Result: a vector of Layer objects, one per Z slice, each containing
 * the 2D polylines for that height.
 *
 * This is the main entry point for mesh slicing: it orchestrates triangle-plane
 * intersection and segment stitching to produce layered toolpaths.
 *
 * @param triangles Triangle mesh to slice
 * @param layer_height Layer height (must be positive)
 * @return Layered paths for each Z slice
 */
inline std::vector<Layer> slice_triangle_mesh_layers(
    const std::vector<Triangle>& triangles, double layer_height
) {
    std::vector<Layer> layers{};
    if (triangles.empty() || layer_height <= 0.) {
        return layers;
    }

    // Compute Z bounds for the mesh
    double min_z = triangles.front().a.GetZ();
    double max_z = triangles.front().a.GetZ();
    for (const auto& tri : triangles) {
        min_z = std::min({min_z, tri.a.GetZ(), tri.b.GetZ(), tri.c.GetZ()});
        max_z = std::max({max_z, tri.a.GetZ(), tri.b.GetZ(), tri.c.GetZ()});
    }

    // Compute the height of the mesh
    const double height = max_z - min_z;
    if (height < 0.) {
        return layers;
    }

    // Calculate the number of layers
    const double tolerance{1e-12};
    const std::size_t layer_count{
        static_cast<std::size_t>(std::floor(height / layer_height + 1. + tolerance))
    };
    layers.reserve(layer_count);

    // For each layer, intersect the triangles with the plane and stitch the segments into polylines
    for (std::size_t i = 0; i < layer_count; ++i) {
        const double z = min_z + static_cast<double>(i) * layer_height;

        // Collect intersection segments for this layer
        std::vector<Segment2D> segments{};
        segments.reserve(triangles.size());
        for (const auto& tri : triangles) {
            Segment2D segment{};
            if (triangle_plane_segment(tri, z, segment, simple_slice::geometry::kEpsilon)) {
                segments.push_back(segment);
            }
        }

        // Store the layer as {z, polylines}
        layers.emplace_back(
            z,
            stitch_segments_into_paths(std::move(segments), simple_slice::geometry::kEpsilon * 10.)
        );
    }
    return layers;
}

}  // namespace simple_slice::slicer
