#include <gtest/gtest.h>

#include "slicer/shapes.hpp"

using namespace simple_slice::slicer;

// Test fixture for shapes tests
class ShapesTest : public ::testing::Test {
protected:
    static constexpr double kTestEpsilon = 1e-9;
};

//----------------------------------------------
// Rectangle tests
//----------------------------------------------

TEST_F(ShapesTest, RectangleValidConstruction) {
    const Rectangle rectangle(0., 0., 8., 6.);

    EXPECT_DOUBLE_EQ(rectangle.min_x, 0.);
    EXPECT_DOUBLE_EQ(rectangle.min_y, 0.);
    EXPECT_DOUBLE_EQ(rectangle.max_x, 8.);
    EXPECT_DOUBLE_EQ(rectangle.max_y, 6.);
}

TEST_F(ShapesTest, RectangleSquare) {
    const Rectangle square(0., 0., 5., 5.);

    EXPECT_DOUBLE_EQ(square.min_x, 0.);
    EXPECT_DOUBLE_EQ(square.min_y, 0.);
    EXPECT_DOUBLE_EQ(square.max_x, 5.);
    EXPECT_DOUBLE_EQ(square.max_y, 5.);
}

TEST_F(ShapesTest, RectangleNegativeCoordinates) {
    const Rectangle rectangle(-5., -3., -1., -1.);

    EXPECT_DOUBLE_EQ(rectangle.min_x, -5.);
    EXPECT_DOUBLE_EQ(rectangle.min_y, -3.);
    EXPECT_DOUBLE_EQ(rectangle.max_x, -1.);
    EXPECT_DOUBLE_EQ(rectangle.max_y, -1.);
}

TEST_F(ShapesTest, RectangleInvalidMinMaxX) {
    EXPECT_THROW(
        {
            const Rectangle rectangle(8., 0., 0., 6.);
        },
        std::invalid_argument
    );
}

TEST_F(ShapesTest, RectangleInvalidMinMaxY) {
    EXPECT_THROW(
        {
            const Rectangle rectangle(0., 6., 8., 0.);
        },
        std::invalid_argument
    );
}

TEST_F(ShapesTest, RectangleDegeneratePoint) {
    const Rectangle point(5., 5., 5., 5.);

    EXPECT_DOUBLE_EQ(point.min_x, 5.);
    EXPECT_DOUBLE_EQ(point.min_y, 5.);
    EXPECT_DOUBLE_EQ(point.max_x, 5.);
    EXPECT_DOUBLE_EQ(point.max_y, 5.);
}

//----------------------------------------------
// Circle tests
//----------------------------------------------

TEST_F(ShapesTest, CircleValidConstruction) {
    const Circle circle(0., 0., 5.);

    EXPECT_DOUBLE_EQ(circle.center_x, 0.);
    EXPECT_DOUBLE_EQ(circle.center_y, 0.);
    EXPECT_DOUBLE_EQ(circle.radius, 5.);
}

TEST_F(ShapesTest, CircleOffsetCenter) {
    const Circle circle(3., 4., 2.5);

    EXPECT_DOUBLE_EQ(circle.center_x, 3.);
    EXPECT_DOUBLE_EQ(circle.center_y, 4.);
    EXPECT_DOUBLE_EQ(circle.radius, 2.5);
}

TEST_F(ShapesTest, CircleInvalidZeroRadius) {
    EXPECT_THROW(
        {
            const Circle circle(0., 0., 0.);
        },
        std::invalid_argument
    );
}

TEST_F(ShapesTest, CircleInvalidNegativeRadius) {
    EXPECT_THROW(
        {
            const Circle circle(0., 0., -1.);
        },
        std::invalid_argument
    );
}

TEST_F(ShapesTest, CircleSmallRadius) {
    const Circle circle(0., 0., 1e-6);

    EXPECT_DOUBLE_EQ(circle.radius, 1e-6);
}

//----------------------------------------------
// Triangle tests
//----------------------------------------------

TEST_F(ShapesTest, TriangleConstruction) {
    const Point a{0., 0., 0.};
    const Point b{1., 0., 0.};
    const Point c{0.5, 1., 0.};

    const Triangle triangle(a, b, c);

    EXPECT_DOUBLE_EQ(triangle.a.GetX(), 0.);
    EXPECT_DOUBLE_EQ(triangle.a.GetY(), 0.);
    EXPECT_DOUBLE_EQ(triangle.b.GetX(), 1.);
    EXPECT_DOUBLE_EQ(triangle.b.GetY(), 0.);
    EXPECT_DOUBLE_EQ(triangle.c.GetX(), 0.5);
    EXPECT_DOUBLE_EQ(triangle.c.GetY(), 1.);
}

TEST_F(ShapesTest, Triangle3D) {
    const Point a{0., 0., 0.};
    const Point b{1., 0., 1.};
    const Point c{0., 1., 2.};

    const Triangle triangle(a, b, c);

    EXPECT_DOUBLE_EQ(triangle.a.GetZ(), 0.);
    EXPECT_DOUBLE_EQ(triangle.b.GetZ(), 1.);
    EXPECT_DOUBLE_EQ(triangle.c.GetZ(), 2.);
}

//----------------------------------------------
// Layer tests
//----------------------------------------------

TEST_F(ShapesTest, LayerConstruction) {
    Path path1;
    path1.emplace_back(0., 0., 0.);
    path1.emplace_back(1., 0., 0.);
    path1.emplace_back(1., 1., 0.);

    Path path2;
    path2.emplace_back(2., 2., 0.);
    path2.emplace_back(3., 2., 0.);

    const Layer layer(1.5, {path1, path2});

    EXPECT_DOUBLE_EQ(layer.z, 1.5);
    EXPECT_EQ(layer.paths.size(), 2U);
    EXPECT_EQ(layer.paths[0].size(), 3U);
    EXPECT_EQ(layer.paths[1].size(), 2U);
}

TEST_F(ShapesTest, LayerEmptyPaths) {
    const Layer layer(0., {});

    EXPECT_DOUBLE_EQ(layer.z, 0.);
    EXPECT_TRUE(layer.paths.empty());
}

TEST_F(ShapesTest, LayerSinglePath) {
    Path path;
    path.emplace_back(0., 0., 0.);
    path.emplace_back(1., 1., 0.);

    const Layer layer(2.5, {path});

    EXPECT_DOUBLE_EQ(layer.z, 2.5);
    EXPECT_EQ(layer.paths.size(), 1U);
    EXPECT_EQ(layer.paths[0].size(), 2U);
}

//----------------------------------------------
// Box type alias tests
//----------------------------------------------

TEST_F(ShapesTest, BoxTypeAlias) {
    // Verify Box is an alias for AxisAlignedBoundingBox
    Box box{0., 0., 0., 1., 1., 1.};

    EXPECT_DOUBLE_EQ(box.min_x, 0.);
    EXPECT_DOUBLE_EQ(box.min_y, 0.);
    EXPECT_DOUBLE_EQ(box.min_z, 0.);
    EXPECT_DOUBLE_EQ(box.max_x, 1.);
    EXPECT_DOUBLE_EQ(box.max_y, 1.);
    EXPECT_DOUBLE_EQ(box.max_z, 1.);
}
