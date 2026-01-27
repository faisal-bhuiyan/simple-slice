#pragma once

/**
 * @file air_hockey_game.hpp
 * @brief Air hockey table model and puck-wall bounce prediction.
 *
 * This module provides a simple air hockey table model and a method to compute the
 * first N wall-contact points of a puck given an initial position and hit angle.
 *
 * Coordinate conventions:
 * - 2D Cartesian table coordinates in the XY plane.
 * - Origin (0,0) is the bottom-left corner of the table.
 * - Table spans x in [0, length], y in [0, width].
 * - Angle is provided in degrees, measured CCW from +x:
 *   0° = +x (right), 90° = +y (up), 180° = -x (left), 270° = -y (down).
 */

#include <array>

#include "geometry/point.hpp"
#include "geometry/utilities.hpp"
#include "geometry/vector.hpp"

namespace simple_slice::air_hockey_game {

using Point = simple_slice::geometry::Point;
using Vector3D = simple_slice::geometry::Vector3D;
constexpr double kEpsilon = simple_slice::geometry::kEpsilon;

/**
 * @brief Air hockey table and bounce simulation (ideal reflections)
 *
 * The table is modeled as an axis-aligned rectangle [0,length]×[0,width].
 * The puck is assumed to move in straight lines at constant speed and reflect
 * elastically off the walls (mirror reflection). No friction/spin/energy loss.
 */
class AirHockey {
public:
    /**
     * @brief Construct an air hockey table
     *
     * @param length Table length in x-direction (must be finite and > 0).
     * @param width  Table width  in y-direction (must be finite and > 0).
     *
     * @throws std::invalid_argument if length/width are non-finite or non-positive.
     */
    AirHockey(double length, double width);

    /**
     * @brief Compute the first 10 wall contact points after a hit
     *
     * @param initial_position Starting position of the puck (must lie within the table bounds).
     * @param angle_deg Hit direction in degrees (CCW from +x).
     *
     * @return An array of 10 points. If the motion becomes degenerate and fewer than
     *         10 hits can be computed, remaining entries will be default-initialized points.
     *
     * @throws std::out_of_range if initial_position is outside [0,length]×[0,width].
     * @throws std::invalid_argument if angle_deg is non-finite (recommended in implementation).
     */
    std::array<Point, 10> puck_hit_locations(const Point& initial_position, double angle_deg) const;

private:
    /// Convert a 2D Point to a 3D vector in the XY plane (z=0)
    static Vector3D ToPositionVector(const Point& p);

    /// Convert a 3D vector in the XY plane (z ignored) to a 2D Point
    static Point ToPoint(const Vector3D& v);

    /**
     * @brief Convert an angle in degrees to a direction vector in the XY plane
     *
     * @param angle_deg Angle in degrees (CCW from +x).
     * @return A direction vector (not necessarily normalized beyond trig identity; z=0).
     */
    static Vector3D DirectionVector(double angle_deg);

    /// Clamp a position to the table bounds to counter floating-point drift
    Vector3D ClampToTable(const Vector3D& position) const;

    /**
     * @brief Compute time parameter to the next vertical wall hit.
     *
     * @param x Current x position
     * @param dx x component of direction
     * @return Smallest positive t such that x + dx*t is 0 or length; or +inf if none.
     */
    double TimeToVerticalWall(double x, double dx) const;

    /**
     * @brief Compute time parameter to the next horizontal wall hit.
     *
     * @param y Current y position
     * @param dy y component of direction
     * @return Smallest positive t such that y + dy*t is 0 or width; or +inf if none.
     */
    double TimeToHorizontalWall(double y, double dy) const;

    double length_;  ///< Table length (x extent), > 0
    double width_;   ///< Table width  (y extent), > 0
};

}  // namespace simple_slice::air_hockey_game
