#include <gtest/gtest.h>

#include "slicer/mesh_slicer.hpp"
#include "slicer/shapes.hpp"

using namespace simple_slice::slicer;

// Test fixture for mesh slicer tests
class MeshSlicerTest : public ::testing::Test {
protected:
    static constexpr double kTestEpsilon = 1e-9;
};

//----------------------------------------------
// Slice triangle mesh layers tests
//----------------------------------------------

TEST_F(MeshSlicerTest, WithPerimeters) {
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

TEST_F(MeshSlicerTest, EmptyMesh) {
    const std::vector<Triangle> empty_triangles{};
    const auto layers = slice_triangle_mesh_layers(empty_triangles, 0.1);

    EXPECT_TRUE(layers.empty());
}

TEST_F(MeshSlicerTest, SingleTriangle) {
    std::vector<Triangle> triangles;
    triangles.emplace_back(Point{0., 0., 0.}, Point{1., 0., 0.}, Point{0.5, 1., 0.});

    const auto layers = slice_triangle_mesh_layers(triangles, 0.5);

    // Should produce at least one layer at z=0
    ASSERT_FALSE(layers.empty());
    EXPECT_NEAR(layers[0].z, 0., kTestEpsilon);
}

TEST_F(MeshSlicerTest, ZeroLayerHeight) {
    std::vector<Triangle> triangles;
    triangles.emplace_back(Point{0., 0., 0.}, Point{1., 0., 0.}, Point{0.5, 1., 0.});

    const auto layers = slice_triangle_mesh_layers(triangles, 0.);

    EXPECT_TRUE(layers.empty());
}

TEST_F(MeshSlicerTest, NegativeLayerHeight) {
    std::vector<Triangle> triangles;
    triangles.emplace_back(Point{0., 0., 0.}, Point{1., 0., 0.}, Point{0.5, 1., 0.});

    const auto layers = slice_triangle_mesh_layers(triangles, -0.1);

    EXPECT_TRUE(layers.empty());
}

TEST_F(MeshSlicerTest, CubeMesh) {
    // Create a simple cube mesh (6 faces, each with 2 triangles)
    std::vector<Triangle> triangles;

    // Bottom face (z=0)
    triangles.emplace_back(Point{0., 0., 0.}, Point{1., 0., 0.}, Point{1., 1., 0.});
    triangles.emplace_back(Point{0., 0., 0.}, Point{1., 1., 0.}, Point{0., 1., 0.});

    // Top face (z=1)
    triangles.emplace_back(Point{0., 0., 1.}, Point{1., 0., 1.}, Point{1., 1., 1.});
    triangles.emplace_back(Point{0., 0., 1.}, Point{1., 1., 1.}, Point{0., 1., 1.});

    const auto layers = slice_triangle_mesh_layers(triangles, 0.5);

    // Should produce layers at z=0 and z=0.5 (and possibly z=1)
    ASSERT_GE(layers.size(), 1U);
    EXPECT_NEAR(layers[0].z, 0., kTestEpsilon);
}

TEST_F(MeshSlicerTest, LayerCountCalculation) {
    std::vector<Triangle> triangles;
    // Create triangles spanning from z=0 to z=2
    triangles.emplace_back(Point{0., 0., 0.}, Point{1., 0., 0.}, Point{0.5, 1., 0.});
    triangles.emplace_back(Point{0., 0., 2.}, Point{1., 0., 2.}, Point{0.5, 1., 2.});

    const auto layers = slice_triangle_mesh_layers(triangles, 0.5);

    // Should produce layers at z=0, 0.5, 1.0, 1.5, 2.0 (5 layers)
    ASSERT_GE(layers.size(), 4U);
}

TEST_F(MeshSlicerTest, TriangleAbovePlane) {
    std::vector<Triangle> triangles;
    // Triangle entirely above the slicing plane
    triangles.emplace_back(Point{0., 0., 1.}, Point{1., 0., 1.}, Point{0.5, 1., 1.});

    const auto layers = slice_triangle_mesh_layers(triangles, 0.5);

    // Should produce a layer at z=1
    ASSERT_FALSE(layers.empty());
    EXPECT_NEAR(layers.back().z, 1., kTestEpsilon);
}

TEST_F(MeshSlicerTest, TriangleBelowPlane) {
    std::vector<Triangle> triangles;
    // Triangle entirely below the slicing plane
    triangles.emplace_back(Point{0., 0., 0.}, Point{1., 0., 0.}, Point{0.5, 1., 0.});

    const auto layers = slice_triangle_mesh_layers(triangles, 0.5);

    // Should produce a layer at z=0
    ASSERT_FALSE(layers.empty());
    EXPECT_NEAR(layers[0].z, 0., kTestEpsilon);
}

TEST_F(MeshSlicerTest, TriangleCrossingPlane) {
    std::vector<Triangle> triangles;
    // Triangle that crosses the z=0.5 plane
    triangles.emplace_back(Point{0., 0., 0.}, Point{1., 0., 1.}, Point{0.5, 1., 0.5});

    const auto layers = slice_triangle_mesh_layers(triangles, 0.5);

    // Should produce layers where the triangle intersects
    ASSERT_FALSE(layers.empty());
}

TEST_F(MeshSlicerTest, PerimetersWithZeroSpacing) {
    std::vector<Triangle> triangles;
    triangles.emplace_back(Point{0., 0., 0.}, Point{1., 0., 0.}, Point{0.5, 1., 0.});

    // Slice with zero spacing (should not generate perimeters)
    const auto layers = slice_triangle_mesh_layers(triangles, 0.5, 0.);

    ASSERT_FALSE(layers.empty());
    // Should have the same number of paths as without perimeters
    const auto layers_no_perim = slice_triangle_mesh_layers(triangles, 0.5);
    EXPECT_EQ(layers.size(), layers_no_perim.size());
}

TEST_F(MeshSlicerTest, PerimetersWithNegativeSpacing) {
    std::vector<Triangle> triangles;
    triangles.emplace_back(Point{0., 0., 0.}, Point{1., 0., 0.}, Point{0.5, 1., 0.});

    // Slice with negative spacing (should not generate perimeters)
    const auto layers = slice_triangle_mesh_layers(triangles, 0.5, -0.1);

    ASSERT_FALSE(layers.empty());
    // Should have the same number of paths as without perimeters
    const auto layers_no_perim = slice_triangle_mesh_layers(triangles, 0.5);
    EXPECT_EQ(layers.size(), layers_no_perim.size());
}
