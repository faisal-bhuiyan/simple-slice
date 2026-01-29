#include <cmath>
#include <numbers>
#include <sstream>
#include <vector>

#include <gtest/gtest.h>

#include "slicer/mesh_slicer.hpp"
#include "slicer/perimeters.hpp"
#include "slicer/shapes.hpp"
#include "slicer/stl_reader.hpp"
#include "slicer/toolpath.hpp"

using simple_slice::slicer::Box;
using simple_slice::slicer::Circle;
using simple_slice::slicer::format_toolpath_gcode;
using simple_slice::slicer::generate_circle_perimeters;
using simple_slice::slicer::generate_layer_perimeters;
using simple_slice::slicer::generate_rectangle_perimeters;
using simple_slice::slicer::Layer;
using simple_slice::slicer::parse_ascii_stl;
using simple_slice::slicer::Path;
using simple_slice::slicer::Point;
using simple_slice::slicer::Rectangle;
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

TEST(Slicer, LayerPerimeters) {
    // Create a layer with multiple paths
    Path path1;
    path1.emplace_back(0., 0., 0.);
    path1.emplace_back(2., 0., 0.);
    path1.emplace_back(2., 2., 0.);
    path1.emplace_back(0., 2., 0.);
    path1.emplace_back(0., 0., 0.);

    Path path2;
    path2.emplace_back(5., 5., 0.);
    path2.emplace_back(7., 5., 0.);
    path2.emplace_back(7., 7., 0.);
    path2.emplace_back(5., 7., 0.);
    path2.emplace_back(5., 5., 0.);

    Layer layer{1.0, {path1, path2}};
    const Layer perim_layer = generate_layer_perimeters(layer, 0.5);

    EXPECT_EQ(perim_layer.z, 1.0);
    EXPECT_GE(perim_layer.paths.size(), 1U);  // Should have at least one perimeter path

    // Verify all paths are closed
    for (const auto& path : perim_layer.paths) {
        ASSERT_GE(path.size(), 2U);
        EXPECT_NEAR(path.front().GetX(), path.back().GetX(), 1e-9);
        EXPECT_NEAR(path.front().GetY(), path.back().GetY(), 1e-9);
    }
}

TEST(Slicer, MeshSlicerWithPerimeters) {
    // Create a simple triangle mesh (a cube face)
    std::vector<Triangle> triangles;
    triangles.emplace_back(Point{0., 0., 0.}, Point{1., 0., 0.}, Point{1., 1., 0.});
    triangles.emplace_back(Point{0., 0., 0.}, Point{1., 1., 0.}, Point{0., 1., 0.});

    // Slice without perimeters
    const auto layers_no_perim = slice_triangle_mesh_layers(triangles, 0.1);
    ASSERT_FALSE(layers_no_perim.empty());

    // Slice with perimeters
    const auto layers_with_perim = slice_triangle_mesh_layers(triangles, 0.1, 0.2);
    ASSERT_FALSE(layers_with_perim.empty());

    // Layers with perimeters should have more paths (or equal) than without
    for (std::size_t i = 0; i < std::min(layers_no_perim.size(), layers_with_perim.size()); ++i) {
        EXPECT_GE(layers_with_perim[i].paths.size(), layers_no_perim[i].paths.size());
    }
}
