#include <gtest/gtest.h>

#include "slicer/perimeters.hpp"
#include "slicer/shapes.hpp"

using namespace simple_slice::slicer;

// Test fixture for perimeters tests
class PerimetersTest : public ::testing::Test {
protected:
    static constexpr double kTestEpsilon = 1e-9;
};

//----------------------------------------------
// Rectangle perimeter tests
//----------------------------------------------

TEST_F(PerimetersTest, RectanglePerimeterCount) {
    const Rectangle rectangle(0., 0., 8., 6.);
    const auto paths = generate_rectangle_perimeters(rectangle, 2.);

    ASSERT_EQ(paths.size(), 2U);
    EXPECT_EQ(paths[0].size(), 5U);
    EXPECT_EQ(paths[1].size(), 5U);
}

TEST_F(PerimetersTest, RectanglePerimeterSingle) {
    const Rectangle rectangle(0., 0., 2., 2.);
    const auto paths = generate_rectangle_perimeters(rectangle, 5.);

    // With spacing 5, only one perimeter should fit
    ASSERT_EQ(paths.size(), 1U);
    EXPECT_EQ(paths[0].size(), 5U);
}

TEST_F(PerimetersTest, RectanglePerimeterZeroSpacing) {
    const Rectangle rectangle(0., 0., 4., 4.);
    const auto paths = generate_rectangle_perimeters(rectangle, 0.);

    EXPECT_TRUE(paths.empty());
}

TEST_F(PerimetersTest, RectanglePerimeterNegativeSpacing) {
    const Rectangle rectangle(0., 0., 4., 4.);
    const auto paths = generate_rectangle_perimeters(rectangle, -1.);

    EXPECT_TRUE(paths.empty());
}

TEST_F(PerimetersTest, RectanglePerimeterClosed) {
    const Rectangle rectangle(0., 0., 4., 4.);
    const auto paths = generate_rectangle_perimeters(rectangle, 1.);

    for (const auto& path : paths) {
        ASSERT_GE(path.size(), 2U);
        // First and last point should be the same (closed path)
        EXPECT_NEAR(path.front().GetX(), path.back().GetX(), kTestEpsilon);
        EXPECT_NEAR(path.front().GetY(), path.back().GetY(), kTestEpsilon);
    }
}

//----------------------------------------------
// Circle perimeter tests
//----------------------------------------------

TEST_F(PerimetersTest, CirclePerimeterCount) {
    const Circle circle(0., 0., 5.);
    const auto paths = generate_circle_perimeters(circle, 2., 8U);

    ASSERT_EQ(paths.size(), 3U);
    EXPECT_EQ(paths[0].size(), 9U);  // 8 segments + 1 closing point
    EXPECT_EQ(paths[1].size(), 9U);
    EXPECT_EQ(paths[2].size(), 9U);
}

TEST_F(PerimetersTest, CirclePerimeterSingle) {
    const Circle circle(0., 0., 2.);
    const auto paths = generate_circle_perimeters(circle, 5., 4U);

    // With spacing 5, only one perimeter should fit
    ASSERT_EQ(paths.size(), 1U);
    EXPECT_EQ(paths[0].size(), 5U);  // 4 segments + 1 closing point
}

TEST_F(PerimetersTest, CirclePerimeterZeroSpacing) {
    const Circle circle(0., 0., 5.);
    const auto paths = generate_circle_perimeters(circle, 0., 8U);

    EXPECT_TRUE(paths.empty());
}

TEST_F(PerimetersTest, CirclePerimeterInvalidSegments) {
    const Circle circle(0., 0., 5.);
    const auto paths = generate_circle_perimeters(circle, 1., 2U);

    // Less than 3 segments should return empty
    EXPECT_TRUE(paths.empty());
}

TEST_F(PerimetersTest, CirclePerimeterClosed) {
    const Circle circle(0., 0., 5.);
    const auto paths = generate_circle_perimeters(circle, 1., 8U);

    for (const auto& path : paths) {
        ASSERT_GE(path.size(), 2U);
        // First and last point should be the same (closed path)
        EXPECT_NEAR(path.front().GetX(), path.back().GetX(), kTestEpsilon);
        EXPECT_NEAR(path.front().GetY(), path.back().GetY(), kTestEpsilon);
    }
}

//----------------------------------------------
// Layer perimeter tests
//----------------------------------------------

TEST_F(PerimetersTest, LayerPerimeters) {
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
        EXPECT_NEAR(path.front().GetX(), path.back().GetX(), kTestEpsilon);
        EXPECT_NEAR(path.front().GetY(), path.back().GetY(), kTestEpsilon);
    }
}

//----------------------------------------------
// Bounding box computation tests
//----------------------------------------------

TEST_F(PerimetersTest, ComputeLayerBoundingBoxEmpty) {
    Layer empty_layer{0., {}};
    const Rectangle bbox = compute_layer_bounding_box(empty_layer);

    EXPECT_DOUBLE_EQ(bbox.min_x, 0.);
    EXPECT_DOUBLE_EQ(bbox.min_y, 0.);
    EXPECT_DOUBLE_EQ(bbox.max_x, 0.);
    EXPECT_DOUBLE_EQ(bbox.max_y, 0.);
}

TEST_F(PerimetersTest, ComputeLayerBoundingBoxSinglePath) {
    Path path;
    path.emplace_back(1., 2., 0.);
    path.emplace_back(3., 4., 0.);
    path.emplace_back(2., 5., 0.);

    Layer layer{0., {path}};
    const Rectangle bbox = compute_layer_bounding_box(layer);

    EXPECT_DOUBLE_EQ(bbox.min_x, 1.);
    EXPECT_DOUBLE_EQ(bbox.min_y, 2.);
    EXPECT_DOUBLE_EQ(bbox.max_x, 3.);
    EXPECT_DOUBLE_EQ(bbox.max_y, 5.);
}

TEST_F(PerimetersTest, ComputeLayerBoundingBoxMultiplePaths) {
    Path path1;
    path1.emplace_back(0., 0., 0.);
    path1.emplace_back(2., 0., 0.);

    Path path2;
    path2.emplace_back(5., 5., 0.);
    path2.emplace_back(7., 7., 0.);

    Layer layer{0., {path1, path2}};
    const Rectangle bbox = compute_layer_bounding_box(layer);

    EXPECT_DOUBLE_EQ(bbox.min_x, 0.);
    EXPECT_DOUBLE_EQ(bbox.min_y, 0.);
    EXPECT_DOUBLE_EQ(bbox.max_x, 7.);
    EXPECT_DOUBLE_EQ(bbox.max_y, 7.);
}

TEST_F(PerimetersTest, ComputeLayerBoundingBoxSinglePoint) {
    Path path;
    path.emplace_back(3., 4., 0.);

    Layer layer{0., {path}};
    const Rectangle bbox = compute_layer_bounding_box(layer);

    EXPECT_DOUBLE_EQ(bbox.min_x, 3.);
    EXPECT_DOUBLE_EQ(bbox.min_y, 4.);
    EXPECT_DOUBLE_EQ(bbox.max_x, 3.);
    EXPECT_DOUBLE_EQ(bbox.max_y, 4.);
}

TEST_F(PerimetersTest, ComputeLayerBoundingBoxNegativeCoordinates) {
    Path path;
    path.emplace_back(-5., -3., 0.);
    path.emplace_back(-1., -1., 0.);

    Layer layer{0., {path}};
    const Rectangle bbox = compute_layer_bounding_box(layer);

    EXPECT_DOUBLE_EQ(bbox.min_x, -5.);
    EXPECT_DOUBLE_EQ(bbox.min_y, -3.);
    EXPECT_DOUBLE_EQ(bbox.max_x, -1.);
    EXPECT_DOUBLE_EQ(bbox.max_y, -1.);
}
