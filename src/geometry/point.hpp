#pragma once

#include <array>
#include <iostream>

namespace simple_slice::geometry {

/**
 * @brief A 3D point in Cartesian coordinates
 */
class Point {
public:
    /**
     * @brief Construct a point from coordinates
     *
     * @param x X coordinate (default: 0)
     * @param y Y coordinate (default: 0)
     * @param z Z coordinate (default: 0)
     */
    Point(double x = 0., double y = 0., double z = 0.) : coords_{x, y, z} {}

    /// Gets the x-coordinate
    double GetX() const { return coords_[0]; }

    /// Gets the y-coordinate
    double GetY() const { return coords_[1]; }

    /// Gets the z-coordinate
    double GetZ() const { return coords_[2]; }

    /**
     * @brief Add two points (component-wise)
     * @param other Point to add
     * @return (x + other.x, y + other.y, z + other.z)
     */
    Point operator+(const Point& other) const {
        return Point{
            this->coords_[0] + other.coords_[0], this->coords_[1] + other.coords_[1],
            this->coords_[2] + other.coords_[2]
        };
    }

    /**
     * @brief Subtract two points (component-wise)
     * @param other Point to subtract
     * @return (x - other.x, y - other.y, z - other.z)
     */
    Point operator-(const Point& other) const {
        return Point{
            this->coords_[0] - other.coords_[0], this->coords_[1] - other.coords_[1],
            this->coords_[2] - other.coords_[2]
        };
    }

    /**
     * @brief Scale a point by a scalar
     * @param s Scalar multiplier
     * @return (x*s, y*s, z*s)
     */
    Point operator*(double s) const {
        return Point{this->coords_[0] * s, this->coords_[1] * s, this->coords_[2] * s};
    }

private:
    std::array<double, 3> coords_;  ///< X, Y, Z coordinates
};

/**
 * @brief Stream insertion operator for Point.
 *
 * Formats and prints a point as `(x, y, z)`.
 *
 * @param os Output stream
 * @param point Point to print
 * @return Reference to the output stream (for chaining)
 */
inline std::ostream& operator<<(std::ostream& output_stream, const Point& point) {
    output_stream << "(" << point.GetX() << ", " << point.GetY() << ", " << point.GetZ() << ")";
    return output_stream;
}

}  // namespace simple_slice::geometry
