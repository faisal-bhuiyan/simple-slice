#pragma once

#include <cmath>

#include "geometry/utilities.hpp"
#include "geometry/vector.hpp"

namespace simple_slice::geometry {

/**
 * @brief Project a point onto an infinite line passing through two points
 *
 * Computes the closest point on the infinite line (a→b) to the query point pt.
 *
 * @param pt Point to project
 * @param a First point defining the line
 * @param b Second point defining the line
 * @return Closest point on the infinite line to pt
 *
 * @note If a and b are coincident (distance < kEpsilon), returns a.
 *       The projected point may lie outside the segment [a,b]; use
 *       `project_point_on_line_segment` if you need to clamp to the segment.
 */
inline Vector3D project_point_on_line(
    const Vector3D& point, const Vector3D& vector_1, const Vector3D& vector_2
) {
    Vector3D line_direction = vector_2 - vector_1;
    double line_length_squared = dot_product(line_direction, line_direction);

    if (std::abs(line_length_squared) <= geometry::kEpsilon) {
        return vector_1;  // Degenerate case: line is a point
    }

    // Compute parameter t such that projection = vector_1 + t * (vector_2 - vector_1)
    double t = dot_product(point - vector_1, line_direction) / line_length_squared;
    return vector_1 + line_direction * t;
}

/**
 * @brief Project a point onto a line segment (clamped to endpoints)
 *
 * Computes the closest point on the closed segment [a,b] to the query point pt.
 * Unlike `project_point_on_line`, the result is guaranteed to lie on [a,b].
 *
 * @param pt Point to project
 * @param a Segment start
 * @param b Segment end
 * @return Closest point on segment [a,b] to pt (may be an endpoint)
 *
 * @note If a and b are coincident (distance < kEpsilon), returns a.
 *       This is the standard "distance to segment" primitive.
 */
inline Vector3D project_point_on_line_segment(
    const Vector3D& point, const Vector3D& vector_1, const Vector3D& vector_2
) {
    Vector3D segment_direction = vector_2 - vector_1;
    double segment_length_squared = dot_product(segment_direction, segment_direction);

    if (std::abs(segment_length_squared) <= kEpsilon) {
        return vector_1;  // Degenerate case: segment is a point
    }

    // Compute parameter t and clamp to [0,1] to stay on the segment
    double t = dot_product(point - vector_1, segment_direction) / segment_length_squared;
    t = clamp_to_unit_interval(t);
    return vector_1 + segment_direction * t;
}

/**
 * @brief 2D orientation (signed area / left-turn test) for three points
 *
 * Computes the signed z-component of the cross product:
 *   orient2D(a,b,c) = cross( b - a, c - a )
 * where the vectors are interpreted in the XY plane.
 *
 * @param a First point
 * @param b Second point
 * @param c Third point
 * @return A signed value proportional to twice the triangle area:
 *         - > 0  : (a,b,c) are in counter-clockwise order (c is left of
 * directed edge a->b)
 *         - < 0  : clockwise order (c is right of a->b)
 *         - = 0  : collinear (within floating-point precision)
 *
 * @note This is a fundamental geometric predicate used for segment
 * intersection, point-in-polygon, convex hull, etc. For robust use with
 * floating-point, compare against an epsilon (e.g., use sign(orient2D(...))).
 */
double orient2D(const Vector3D& a, const Vector3D& b, const Vector3D& c) {
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

/**
 * @brief 2D axis-aligned bounding box (AABB) in the XY plane.
 *
 * Stores minimum and maximum x/y extents. AABBs are widely used for cheap
 * containment tests and early-out rejection in intersection / distance queries.
 *
 * @note This struct is "2D" even though callers may pass `Vector3D`; only x/y
 * are used.
 */
struct AxisAlignedBoundingBox2D {
    double min_x, min_y;
    double max_x, max_y;
};

/**
 * @brief Compute the 2D axis-aligned bounding box of the segment [a,b] in the
 * XY plane.
 *
 * The resulting box is optionally expanded by @p pad in all directions to
 * account for floating-point error and to support "inclusive" geometric
 * predicates.
 *
 * @param a First endpoint (uses a.x,a.y)
 * @param b Second endpoint (uses b.x,b.y)
 * @param pad Non-negative padding added to all sides (defaults to `kEpsilon`)
 * @return The padded AABB that contains both endpoints.
 */
inline AxisAlignedBoundingBox2D axis_aligned_bounding_box_2d(
    const Vector3D& a, const Vector3D& b, double pad = kEpsilon
) {
    AxisAlignedBoundingBox2D box{0., 0., 0., 0.};  // Initialize to 0,0,0,0
    // Calculate the bounding box of the segment
    box.min_x = std::min(a.x, b.x) - pad;
    box.max_x = std::max(a.x, b.x) + pad;
    box.min_y = std::min(a.y, b.y) - pad;
    box.max_y = std::max(a.y, b.y) + pad;
    return box;
}

/**
 * @brief Test whether a point lies inside (or on the boundary of) a 2D AABB in
 * the XY plane.
 *
 * @param box Axis-aligned bounding box (min/max extents)
 * @param point Query point (uses point.x, point.y)
 * @return true if point is within [min_x,max_x] and [min_y,max_y], else false.
 *
 * @note If the box was built with padding (e.g., `kEpsilon`), this predicate
 * becomes tolerant to small numerical error, which is useful for segment
 * intersection logic.
 */
inline bool contains_point_2d(const AxisAlignedBoundingBox2D& box, const Vector3D& point) {
    return (
        point.x >= box.min_x && point.x <= box.max_x && point.y >= box.min_y && point.y <= box.max_y
    );
}

/**
 * @brief Test whether a point lies on the closed line segment [a,b] in 2D (XY plane)
 *
 * The test has two parts:
 * 1) **Collinearity**: `orient2D(a,b,p)` must be ~0 (within `kEpsilon`), meaning a, b, and p lie on
 * the same infinite line.
 * 2) **Bounding box**: p's x and y coordinates must lie within the axis-aligned bounding box of a
 * and b (expanded by `kEpsilon` to tolerate floating error).
 *
 * @param vector_1 Segment start
 * @param vector_2 Segment end
 * @param point Query point
 * @return true if p is on the segment [a,b] (including endpoints), false
 * otherwise.
 *
 * @note This helper is typically used for robust segment–segment intersection
 * to handle collinear and endpoint-touching cases.
 */
inline bool on_line_segment_2d(
    const Vector3D& vector_1, const Vector3D& vector_2, const Vector3D& point
) {
    // 1) Must be (approximately) collinear with the supporting line through [vector_1, vector_2]
    if (std::abs(orient2D(vector_1, vector_2, point)) > kEpsilon) {
        return false;
    }

    // 2) Must lie within the (padded) bounding box of the segment endpoints
    const auto box = axis_aligned_bounding_box_2d(vector_1, vector_2, kEpsilon);
    return contains_point_2d(box, point);
}

/**
 * @brief Test whether two closed line segments [@p vector_1, @p vector_2] and [@p vector_3, @p
 * vector_4] intersect in 2D (XY plane).
 *
 * Uses orientation tests to detect a proper intersection (endpoints on opposite
 * sides), and falls back to `on_line_segment_2d` to handle collinear overlap and
 * endpoint-touching.
 *
 * @return true if segments intersect (including touching/overlap), false
 * otherwise.
 */
inline bool line_segments_intersect_2d(
    const Vector3D& vector_1, const Vector3D& vector_2, const Vector3D& vector_3,
    const Vector3D& vector_4
) {
    const int ab_c = sign(orient2D(vector_1, vector_2, vector_3), kEpsilon);
    const int ab_d = sign(orient2D(vector_1, vector_2, vector_4), kEpsilon);
    const int cd_a = sign(orient2D(vector_3, vector_4, vector_1), kEpsilon);
    const int cd_b = sign(orient2D(vector_3, vector_4, vector_2), kEpsilon);

    // Proper intersection
    if (ab_c * ab_d < 0 && cd_a * cd_b < 0) {
        return true;
    }

    // Collinear / touching cases
    if (ab_c == 0 && on_line_segment_2d(vector_1, vector_2, vector_3)) {
        return true;
    }
    if (ab_d == 0 && on_line_segment_2d(vector_1, vector_2, vector_4)) {
        return true;
    }
    if (cd_a == 0 && on_line_segment_2d(vector_3, vector_4, vector_1)) {
        return true;
    }
    if (cd_b == 0 && on_line_segment_2d(vector_3, vector_4, vector_2)) {
        return true;
    }

    // No intersection
    return false;
}

}  // namespace simple_slice::geometry
