#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "geometry/point.hpp"
#include "geometry/utilities.hpp"
#include "geometry/vector.hpp"

namespace simple_slice::geometry {

namespace {

//----------------------------------------------------
// Conversions
//----------------------------------------------------

/// Converts a point to a vector
inline Vector3D to_vector(const Point& p) {
    return Vector3D{p.GetX(), p.GetY(), p.GetZ()};
}

/// Converts a vector to a point
inline Point to_point(const Vector3D& v) {
    return Point{v.x, v.y, v.z};
}

}  // namespace

//----------------------------------------------------
// Projections
//----------------------------------------------------

/**
 * @brief Project a point onto an infinite line passing through two points
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
    const Vector3D projection = a_vec + line_direction * t;
    return to_point(projection);
}

/**
 * @brief Project a point onto a line segment (clamped to endpoints)
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

    const Vector3D projection = a_vec + segment_direction * t;
    return to_point(projection);
}

/**
 * @brief 2D orientation (signed area / left-turn test) for three points
 *
 * Computes the signed area of the triangle formed by points a, b, and c in the XY plane:
 *   signed_area_2D(a,b,c) = cross( b - a, c - a ).z
 * where vectors are interpreted in the XY plane.
 *
 * - A positive value means c is to the left of segment a->b (i.e. the triangle a->b->c is
 * counter-clockwise).
 * - A negative value means c is to the right of segment a->b (i.e. the triangle a->b->c is
 * clockwise).
 * - Zero means the points are co-linear.
 */
inline double signed_area_2D(const Point& a, const Point& b, const Point& c) {
    const Vector3D a_vec = to_vector(a);
    const Vector3D b_vec = to_vector(b);
    const Vector3D c_vec = to_vector(c);

    // Compute the cross product of the vectors ab and ac
    const Vector3D ab = b_vec - a_vec;
    const Vector3D ac = c_vec - a_vec;
    const Vector3D cross = cross_product(ab, ac);

    // The signed area is the z-component of the cross product
    return cross.z;
}

//----------------------------------------------------
// Axis aligned bounding box
//----------------------------------------------------

/**
 * @brief 3D axis-aligned bounding box (AABB)
 */
struct AxisAlignedBoundingBox {
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;

    AxisAlignedBoundingBox(
        double min_x_in = 0., double min_y_in = 0., double min_z_in = 0., double max_x_in = 0.,
        double max_y_in = 0., double max_z_in = 0.
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
};

/**
 * @brief Compute the 3D axis-aligned bounding box of the segment [a,b]
 */
inline AxisAlignedBoundingBox axis_aligned_bounding_box(
    const Point& a, const Point& b, double pad = kEpsilon
) {
    AxisAlignedBoundingBox box{0., 0., 0., 0., 0., 0.};
    box.min_x = std::min(a.GetX(), b.GetX()) - pad;
    box.max_x = std::max(a.GetX(), b.GetX()) + pad;
    box.min_y = std::min(a.GetY(), b.GetY()) - pad;
    box.max_y = std::max(a.GetY(), b.GetY()) + pad;
    box.min_z = std::min(a.GetZ(), b.GetZ()) - pad;
    box.max_z = std::max(a.GetZ(), b.GetZ()) + pad;
    return box;
}

//----------------------------------------------------
// Containment
//----------------------------------------------------

/**
 * @brief Test whether a point lies inside (or on the boundary of) a 3D AABB
 */
inline bool contains_point_3d(const AxisAlignedBoundingBox& box, const Point& point) {
    return (
        point.GetX() >= box.min_x && point.GetX() <= box.max_x && point.GetY() >= box.min_y &&
        point.GetY() <= box.max_y && point.GetZ() >= box.min_z && point.GetZ() <= box.max_z
    );
}

/**
 * @brief Test whether a point lies on the closed line segment [a,b] in 3D
 */
inline bool on_line_segment_3d(const Point& a, const Point& b, const Point& point) {
    // Check if the point is on the line segment
    const Vector3D a_vec = to_vector(a);
    const Vector3D b_vec = to_vector(b);
    const Vector3D p_vec = to_vector(point);

    const Vector3D ab = b_vec - a_vec;
    const Vector3D ap = p_vec - a_vec;
    const Vector3D cross = cross_product(ab, ap);
    if (magnitude(cross) > kEpsilon) {
        return false;
    }

    // Check if the point is inside the bounding box
    const auto box = axis_aligned_bounding_box(a, b, kEpsilon);
    return contains_point_3d(box, point);
}

//----------------------------------------------------
// Intersections
//----------------------------------------------------

/**
 * @brief Test whether two closed line segments [a,b] and [c,d] intersect in 2D (XY plane)
 */
inline bool line_segments_intersect_2d(
    const Point& a, const Point& b, const Point& c, const Point& d
) {
    const int ab_c = sign(signed_area_2D(a, b, c), kEpsilon);
    const int ab_d = sign(signed_area_2D(a, b, d), kEpsilon);
    const int cd_a = sign(signed_area_2D(c, d, a), kEpsilon);
    const int cd_b = sign(signed_area_2D(c, d, b), kEpsilon);

    // Proper intersection
    if (ab_c * ab_d < 0 && cd_a * cd_b < 0) {
        return true;
    }

    // Co-linear/touching cases
    if (ab_c == 0 && on_line_segment_3d(a, b, c)) {
        return true;
    }
    if (ab_d == 0 && on_line_segment_3d(a, b, d)) {
        return true;
    }
    if (cd_a == 0 && on_line_segment_3d(c, d, a)) {
        return true;
    }
    if (cd_b == 0 && on_line_segment_3d(c, d, b)) {
        return true;
    }

    return false;
}

}  // namespace simple_slice::geometry
