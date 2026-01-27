#pragma once

#include <cmath>
#include <iostream>

namespace simple_slice::geometry {

/*
 * A simple class to represent a 2D point in Cartesian coordinates
 */
class Point {
public:
    Point() : x_{0.}, y_{0.} {}
    Point(double x, double y) : x_{x}, y_{y} {}

    /// Get the x-coordinate of the point
    double GetX() const { return x_; }

    /// Get the y-coordinate of the point
    double GetY() const { return y_; }

    Point operator+(const Point& other) const { return Point{x_ + other.x_, y_ + other.y_}; }
    Point operator-(const Point& other) const { return Point{x_ - other.x_, y_ - other.y_}; }
    Point operator*(double s) const { return Point{x_ * s, y_ * s}; }
    Point operator/(double s) const { return Point{x_ / s, y_ / s}; }

private:
    double x_;  //< x-coordinate of the point
    double y_;  //< y-coordinate of the point
};

/*
 * Overload the << operator to print the point
 */
inline std::ostream& operator<<(std::ostream& os, const Point& p) {
    os << "(" << p.GetX() << ", " << p.GetY() << ")";
    return os;
}

}  // namespace simple_slice::geometry
