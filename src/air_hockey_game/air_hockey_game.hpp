#pragma once

#include <array>

#include "geometry/point.hpp"
#include "geometry/utilities.hpp"
#include "geometry/vector.hpp"

namespace simple_slice::air_hockey_game {

using Point = simple_slice::geometry::Point;
using Vector3D = simple_slice::geometry::Vector3D;
constexpr double kEpsilon = simple_slice::geometry::kEpsilon;

/**
 * @brief Air hockey table and puck-table contact simulation w/ ideal reflections
 *
 * This class provides a simple air hockey table model and a method to compute the
 * first N wall-contact points of a puck given an initial position and hit angle.
 *
 * Coordinate conventions:
 * - 2D Cartesian table coordinates in the XY plane.
 * - Origin (0,0) is the bottom-left corner of the table.
 * - Table spans x in [0, length], y in [0, width].
 * - Angle is provided in degrees, measured CCW from +x:
 *   0° = +x (right), 90° = +y (up), 180° = -x (left), 270° = -y (down).
 *
 * Coordinate system (top view):
 *
 *     y ↑
 *       (0, W)                (L, W)
 *       +--------------------+
 *       |                    |
 *       |   θ (ccw from +x)  |
 *       |    ↗               |
 *       |   • P(x,y)         |
 *       |                    |
 *       +--------------------+
 *      (0,0)                 (L, 0)    --> x
 *
 * ASSUMPTIONS:
 *
 * - The table is modeled as an axis-aligned rectangle [0, length] x [0, width].
 * - The puck is assumed to move in straight lines at CONSTANT speed (i.e. no acceleration) and
 *   reflect elastically off the walls (i.e. mirror reflection).
 * - We are assuming the magnitude of velocity/speed is 1 unit/second here.
 * - No friction/spin or any other energy loss in the system.
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

    /**
     * @brief Print the table as an ASCII schematic
     *
     * @param hits Array of hit points
     * @param cols Number of columns
     * @param rows Number of rows
     */
    void PrintTable(const std::array<Point, 10>& hits, int cols = 60, int rows = 20) const;

private:
    double length_;  ///< length of table
    double width_;   ///< width of table

    /**
     * @brief Converts an angle in degrees to a direction vector
     *
     * @param angle Angle in degrees (CCW from +x)
     * @return A direction vector
     */
    static Vector3D DirectionVector(double angle);

    /// Clamp a position to the table bounds to counter floating-point drift
    Vector3D ClampToTable(const Vector3D& position) const;

    /**
     * @brief Computes the forward time parameter t to the next wall along one axis.
     *
     * @param position Current position (x or y)
     * @param velocity Velocity component along that axis
     * @param min_bound Minimum bound
     * @param max_bound Maximum bound
     * @return Smallest positive t such that (position + direction * t) is at the wall; or +inf if
     * none
     */
    double TimeToNextWall(double position, double velocity, double min_bound, double max_bound)
        const;
};

}  // namespace simple_slice::air_hockey_game
