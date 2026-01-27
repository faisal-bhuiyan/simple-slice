#pragma once

#include <algorithm>
#include <cmath>

#include "geometry/point.hpp"
#include "geometry/utilities.hpp"
#include "geometry/vector.hpp"

namespace simple_slice::geometry {

namespace {

/// Converts a point to a vector
inline Vector3D to_vector(const Point& p) {
    return Vector3D{p.GetX(), p.GetY(), p.GetZ()};
}

/// Converts a vector to a point
inline Point to_point(const Vector3D& v) {
    return Point{v.x, v.y, v.z};
}

}  // namespace

/**
 * @brief Project a point onto an infinite line passing through two points.
 *
 * Computes the closest point on the infinite line (a→b) to the query point.
 *
 * @param point Point to project
 * @param a First point defining the line
 * @param b Second point defining the line
 * @return Closest point on the infinite line to point
 *
 * @note If a and b are coincident (distance < kEpsilon), returns a.
 *       The projected point may lie outside the segment [a,b]; use
 *       `project_point_on_line_segment` if you need to clamp to the segment.
 */
inline Point project_point_on_line(const Point& point, const Point& a, const Point& b) {
    const Vector3D a_vec = to_vector(a);
    const Vector3D b_vec = to_vector(b);
    const Vector3D p_vec = to_vector(point);

    const Vector3D line_direction = b_vec - a_vec;
    const double line_length_squared = dot_product(line_direction, line_direction);

    if (std::abs(line_length_squared) <= kEpsilon) {
        return a;  // Degenerate case: line is a point
    }

    const double t = dot_product(p_vec - a_vec, line_direction) / line_length_squared;
    const Vector3D proj = a_vec + line_direction * t;
    return to_point(proj);
}

/**
 * @brief Project a point onto a line segment (clamped to endpoints).
 *
 * Computes the closest point on the closed segment [a,b] to the query point.
 *
 * @param point Point to project
 * @param a Segment start
 * @param b Segment end
 * @return Closest point on segment [a,b] to point (may be an endpoint)
 *
 * @note If a and b are coincident (distance < kEpsilon), returns a.
 *       This is the standard "distance to segment" primitive.
 */
inline Point project_point_on_line_segment(const Point& point, const Point& a, const Point& b) {
    const Vector3D a_vec = to_vector(a);
    const Vector3D b_vec = to_vector(b);
    const Vector3D p_vec = to_vector(point);

    const Vector3D segment_direction = b_vec - a_vec;
    const double segment_length_squared = dot_product(segment_direction, segment_direction);

    if (std::abs(segment_length_squared) <= kEpsilon) {
        return a;  // Degenerate case: segment is a point
    }

    double t = dot_product(p_vec - a_vec, segment_direction) / segment_length_squared;
    t = clamp_to_unit_interval(t);

    const Vector3D proj = a_vec + segment_direction * t;
    return to_point(proj);
}

/**
 * @brief 2D orientation (signed area / left-turn test) for three points.
 *
 * Computes the signed z-component of the cross product:
 *   orient2D(a,b,c) = cross( b - a, c - a )
 * where vectors are interpreted in the XY plane.
 */
inline double orient2D(const Point& a, const Point& b, const Point& c) {
    const Vector3D a_vec = to_vector(a);
    const Vector3D b_vec = to_vector(b);
    const Vector3D c_vec = to_vector(c);

    return (b_vec.x - a_vec.x) * (c_vec.y - a_vec.y) - (b_vec.y - a_vec.y) * (c_vec.x - a_vec.x);
}

/**
 * @brief 2D axis-aligned bounding box (AABB) in the XY plane.
 */
struct AxisAlignedBoundingBox2D {
    double min_x, min_y;
    double max_x, max_y;
};

/**
 * @brief Compute the 2D axis-aligned bounding box of the segment [a,b].
 */
inline AxisAlignedBoundingBox2D axis_aligned_bounding_box_2d(
    const Point& a, const Point& b, double pad = kEpsilon
) {
    AxisAlignedBoundingBox2D box{0., 0., 0., 0.};
    box.min_x = std::min(a.GetX(), b.GetX()) - pad;
    box.max_x = std::max(a.GetX(), b.GetX()) + pad;
    box.min_y = std::min(a.GetY(), b.GetY()) - pad;
    box.max_y = std::max(a.GetY(), b.GetY()) + pad;
    return box;
}

/**
 * @brief Test whether a point lies inside (or on the boundary of) a 2D AABB.
 */
inline bool contains_point_2d(const AxisAlignedBoundingBox2D& box, const Point& point) {
    return (
        point.GetX() >= box.min_x && point.GetX() <= box.max_x && point.GetY() >= box.min_y &&
        point.GetY() <= box.max_y
    );
}

/**
 * @brief Test whether a point lies on the closed line segment [a,b] in 2D.
 */
inline bool on_line_segment_2d(const Point& a, const Point& b, const Point& point) {
    if (std::abs(orient2D(a, b, point)) > kEpsilon) {
        return false;
    }
    const auto box = axis_aligned_bounding_box_2d(a, b, kEpsilon);
    return contains_point_2d(box, point);
}

/**
 * @brief Test whether two closed line segments [a,b] and [c,d] intersect in 2D (XY plane).
 */
inline bool line_segments_intersect_2d(
    const Point& a, const Point& b, const Point& c, const Point& d
) {
    const int ab_c = sign(orient2D(a, b, c), kEpsilon);
    const int ab_d = sign(orient2D(a, b, d), kEpsilon);
    const int cd_a = sign(orient2D(c, d, a), kEpsilon);
    const int cd_b = sign(orient2D(c, d, b), kEpsilon);

    // Proper intersection
    if (ab_c * ab_d < 0 && cd_a * cd_b < 0) {
        return true;
    }

    // Collinear / touching cases
    if (ab_c == 0 && on_line_segment_2d(a, b, c))
        return true;
    if (ab_d == 0 && on_line_segment_2d(a, b, d))
        return true;
    if (cd_a == 0 && on_line_segment_2d(c, d, a))
        return true;
    if (cd_b == 0 && on_line_segment_2d(c, d, b))
        return true;

    return false;
}

}  // namespace simple_slice::geometry
