#pragma once

#include <stdexcept>
#include <vector>

#include "geometry/point.hpp"
#include "geometry/projections.hpp"

namespace simple_slice::slicer {

using Point = simple_slice::geometry::Point;
using Path = std::vector<Point>;

//----------------------------------------------------
// shapes used in 2D slicing
//----------------------------------------------------

/**
 * @brief Axis-aligned rectangle in 2D (XY plane)
 */
struct Rectangle {
    double min_x;  //< minimum x coordinate
    double min_y;  //< minimum y coordinate
    double max_x;  //< maximum x coordinate
    double max_y;  //< maximum y coordinate

    /**
     * @brief Constructor for Rectangle
     */
    Rectangle(double min_x_in, double min_y_in, double max_x_in, double max_y_in)
        : min_x(min_x_in), min_y(min_y_in), max_x(max_x_in), max_y(max_y_in) {
        if (min_x > max_x || min_y > max_y) {
            throw std::invalid_argument("Rectangle: min must be <= max on all axes");
        }
    }
};

/**
 * @brief Circle in 2D (XY plane)
 */
struct Circle {
    double center_x;  //< x coordinate of the center
    double center_y;  //< y coordinate of the center
    double radius;    //< radius of the circle

    /**
     * @brief Constructor for Circle
     * @param center_x_in x coordinate of the center input
     * @param center_y_in y coordinate of the center input
     * @param radius_in radius input
     */
    Circle(double center_x_in, double center_y_in, double radius_in)
        : center_x(center_x_in), center_y(center_y_in), radius(radius_in) {
        if (radius <= 0.) {
            throw std::invalid_argument("Circle: radius must be positive");
        }
    }
};

//----------------------------------------------------
// shapes used in 3D slicing
//----------------------------------------------------

/**
 * @brief Axis-aligned box in 3D (XYZ)
 */
using Box = simple_slice::geometry::AxisAlignedBoundingBox;

/**
 * @brief Triangle in 3D for mesh slicing
 */
struct Triangle {
    Point a;  //< first vertex
    Point b;  //< second vertex
    Point c;  //< third vertex

    /**
     * @brief Constructor for Triangle
     * @param a_in first vertex input
     * @param b_in second vertex input
     * @param c_in third vertex input
     */
    Triangle(const Point& a_in, const Point& b_in, const Point& c_in) : a(a_in), b(b_in), c(c_in) {}
};

/**
 * @brief A single Z layer containing 2D paths
 *
 * z is the layer height
 * paths are polylines in XY plane
 */
struct Layer {
    double z;                 //< height of the layer
    std::vector<Path> paths;  //< paths in the layer

    /**
     * @brief Constructor for Layer
     * @param z_in height of the layer input
     * @param paths_in paths in the layer input
     */
    Layer(double z_in, std::vector<Path> paths_in) : z(z_in), paths(std::move(paths_in)) {}
};

}  // namespace simple_slice::slicer
