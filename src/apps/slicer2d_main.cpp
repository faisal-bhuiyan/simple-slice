#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "slicer/perimeters.hpp"
#include "slicer/shapes.hpp"
#include "slicer/toolpath.hpp"

int main() {
    using simple_slice::slicer::Circle;
    using simple_slice::slicer::format_toolpath_gcode;
    using simple_slice::slicer::generate_circle_perimeters;
    using simple_slice::slicer::generate_rectangle_perimeters;
    using simple_slice::slicer::Path;
    using simple_slice::slicer::Rectangle;

    //----------------------------------------------------
    // Rectangle demo (XY plane)
    // Edit the values below to change the output geometry
    //----------------------------------------------------

    double rect_min_x{0.};  // rectangle min X
    double rect_min_y{0.};  // rectangle min Y
    double rect_max_x{8.};  // rectangle max X
    double rect_max_y{6.};  // rectangle max Y
    double spacing{0.5};    // perimeter spacing (mm)

    if (rect_max_x <= rect_min_x || rect_max_y <= rect_min_y) {
        std::cerr << "Rectangle max must be greater than min.\n";
        return 1;
    }
    if (spacing <= 0.) {
        std::cerr << "Spacing must be positive.\n";
        return 1;
    }

    const Rectangle rectangle(rect_min_x, rect_min_y, rect_max_x, rect_max_y);
    const std::vector<Path> rect_paths = generate_rectangle_perimeters(rectangle, spacing);

    std::ofstream rect_out("slicer2d_rectangle.gcode");
    if (!rect_out) {
        std::cerr << "Failed to open slicer2d_rectangle.gcode for writing.\n";
        return 1;
    }
    rect_out << format_toolpath_gcode(rect_paths);

    //----------------------------------------------------
    // Circle demo (XY plane)
    // Edit the values below to change the output geometry
    //----------------------------------------------------

    double circle_center_x{0.};  // circle center X
    double circle_center_y{0.};  // circle center Y
    double circle_radius{6.};    // circle radius
    std::size_t segments{16};    // polygon segments per perimeter

    if (circle_radius <= 0.) {
        std::cerr << "Circle radius must be positive.\n";
        return 1;
    }
    if (segments < 3) {
        std::cerr << "Segments must be >= 3.\n";
        return 1;
    }

    const Circle circle(circle_center_x, circle_center_y, circle_radius);
    const std::vector<Path> circle_paths = generate_circle_perimeters(circle, spacing, segments);

    std::ofstream circle_out("slicer2d_circle.gcode");
    if (!circle_out) {
        std::cerr << "Failed to open slicer2d_circle.gcode for writing.\n";
        return 1;
    }
    circle_out << format_toolpath_gcode(circle_paths);

    return 0;
}
