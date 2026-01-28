#include <cmath>
#include <sstream>
#include <vector>

#include <gtest/gtest.h>

#include "slicer/mesh_slicer.hpp"
#include "slicer/perimeters.hpp"
#include "slicer/shapes.hpp"
#include "slicer/slicer3d.hpp"
#include "slicer/stl_reader.hpp"
#include "slicer/toolpath.hpp"

using simple_slice::slicer::Box;
using simple_slice::slicer::Circle;
using simple_slice::slicer::format_toolpath_gcode;
using simple_slice::slicer::generate_circle_perimeters;
using simple_slice::slicer::generate_rectangle_perimeters;
using simple_slice::slicer::parse_ascii_stl;
using simple_slice::slicer::Path;
using simple_slice::slicer::Point;
using simple_slice::slicer::Rectangle;
using simple_slice::slicer::slice_box_layers;
using simple_slice::slicer::slice_triangle_mesh_layers;
using simple_slice::slicer::Triangle;

TEST(Slicer, RectanglePerimeterCount) {
    const Rectangle rectangle(0., 0., 8., 6.);
    const auto paths = generate_rectangle_perimeters(rectangle, 2.);
    ASSERT_EQ(paths.size(), 2U);
    EXPECT_EQ(paths[0].size(), 5U);
    EXPECT_EQ(paths[1].size(), 5U);
}

TEST(Slicer, CirclePerimeterCount) {
    const Circle circle(0., 0., 5.);
    const auto paths = generate_circle_perimeters(circle, 2., 8U);
    ASSERT_EQ(paths.size(), 3U);
    EXPECT_EQ(paths[0].size(), 9U);
    EXPECT_EQ(paths[1].size(), 9U);
    EXPECT_EQ(paths[2].size(), 9U);
}

TEST(Slicer, ToolpathOutputFormat) {
    const Rectangle rectangle(0., 0., 4., 4.);
    const auto paths = generate_rectangle_perimeters(rectangle, 2.);
    const std::string output = format_toolpath_gcode(paths);

    ASSERT_FALSE(output.empty());
    EXPECT_NE(output.find("G0"), std::string::npos);
    EXPECT_NE(output.find("G1"), std::string::npos);

    // Each path emits a G0 + G1 lines for the remaining points
    const std::size_t expected_lines = paths[0].size();
    std::size_t line_count = 0;
    for (char ch : output) {
        if (ch == '\n') {
            ++line_count;
        }
    }
    EXPECT_GE(line_count, expected_lines);
}
