#include <cmath>

#include <gtest/gtest.h>

#include "geometry/point.hpp"
#include "geometry/projections.hpp"
#include "geometry/utilities.hpp"

using namespace simple_slice::geometry;

class ProjectionsTest : public ::testing::Test {
protected:
    static constexpr double kTestEpsilon = 1e-9;

    void ExpectPointNear(const Point& actual, const Point& expected) {
        EXPECT_NEAR(actual.GetX(), expected.GetX(), kTestEpsilon);
        EXPECT_NEAR(actual.GetY(), expected.GetY(), kTestEpsilon);
        EXPECT_NEAR(actual.GetZ(), expected.GetZ(), kTestEpsilon);
    }
};

//----------------------------------------------
// project_point_on_line() tests
//----------------------------------------------

TEST_F(ProjectionsTest, ProjectPointOnLineOrthogonal) {
    Point point{1., 1., 0.};
    Point a{0., 0., 0.};
    Point b{2., 0., 0.};

    Point result = project_point_on_line(point, a, b);

    ExpectPointNear(result, Point{1., 0., 0.});
}

TEST_F(ProjectionsTest, ProjectPointOnLineAlreadyOnLine) {
    Point point{1., 0., 0.};
    Point a{0., 0., 0.};
    Point b{2., 0., 0.};

    Point result = project_point_on_line(point, a, b);

    ExpectPointNear(result, point);
}

TEST_F(ProjectionsTest, ProjectPointOnLineBeyondSegment) {
    Point point{3., 1., 0.};
    Point a{0., 0., 0.};
    Point b{2., 0., 0.};

    Point result = project_point_on_line(point, a, b);

    ExpectPointNear(result, Point{3., 0., 0.});
}

TEST_F(ProjectionsTest, ProjectPointOnLineBeforeSegment) {
    Point point{-1., 1., 0.};
    Point a{0., 0., 0.};
    Point b{2., 0., 0.};

    Point result = project_point_on_line(point, a, b);

    ExpectPointNear(result, Point{-1., 0., 0.});
}

TEST_F(ProjectionsTest, ProjectPointOnLineDiagonal) {
    Point point{2., 0., 0.};
    Point a{0., 0., 0.};
    Point b{1., 1., 0.};

    Point result = project_point_on_line(point, a, b);

    ExpectPointNear(result, Point{1., 1., 0.});
}

TEST_F(ProjectionsTest, ProjectPointOnLineDegenerateCase) {
    Point point{1., 1., 0.};
    Point a{0.5, 0.5, 0.};
    Point b{0.5, 0.5, 0.};

    Point result = project_point_on_line(point, a, b);

    ExpectPointNear(result, a);
}

//----------------------------------------------
// project_point_on_line_segment() tests
//----------------------------------------------

TEST_F(ProjectionsTest, ProjectPointOnSegmentOrthogonal) {
    Point point{1., 1., 0.};
    Point a{0., 0., 0.};
    Point b{2., 0., 0.};

    Point result = project_point_on_line_segment(point, a, b);

    ExpectPointNear(result, Point{1., 0., 0.});
}

TEST_F(ProjectionsTest, ProjectPointOnSegmentClampedToStart) {
    Point point{-1., 1., 0.};
    Point a{0., 0., 0.};
    Point b{2., 0., 0.};

    Point result = project_point_on_line_segment(point, a, b);

    ExpectPointNear(result, a);
}

TEST_F(ProjectionsTest, ProjectPointOnSegmentClampedToEnd) {
    Point point{3., 1., 0.};
    Point a{0., 0., 0.};
    Point b{2., 0., 0.};

    Point result = project_point_on_line_segment(point, a, b);

    ExpectPointNear(result, b);
}

TEST_F(ProjectionsTest, ProjectPointOnSegmentAtStart) {
    Point point{0., 1., 0.};
    Point a{0., 0., 0.};
    Point b{2., 0., 0.};

    Point result = project_point_on_line_segment(point, a, b);

    ExpectPointNear(result, a);
}

TEST_F(ProjectionsTest, ProjectPointOnSegmentAtEnd) {
    Point point{2., 1., 0.};
    Point a{0., 0., 0.};
    Point b{2., 0., 0.};

    Point result = project_point_on_line_segment(point, a, b);

    ExpectPointNear(result, b);
}

TEST_F(ProjectionsTest, ProjectPointOnSegmentDegenerateCase) {
    Point point{1., 1., 0.};
    Point a{0.5, 0.5, 0.};
    Point b{0.5, 0.5, 0.};

    Point result = project_point_on_line_segment(point, a, b);

    ExpectPointNear(result, a);
}

//----------------------------------------------
// signed_area_2D() tests
//----------------------------------------------

TEST_F(ProjectionsTest, signed_area_2DCounterclockwise) {
    Point a{0., 0., 0.};
    Point b{1., 0., 0.};
    Point c{0.5, 1., 0.};

    double result = signed_area_2D(a, b, c);

    EXPECT_GT(result, 0.);
}

TEST_F(ProjectionsTest, signed_area_2DClockwise) {
    Point a{0., 0., 0.};
    Point b{1., 0., 0.};
    Point c{0.5, -1., 0.};

    double result = signed_area_2D(a, b, c);

    EXPECT_LT(result, 0.);
}

TEST_F(ProjectionsTest, signed_area_2DCollinear) {
    Point a{0., 0., 0.};
    Point b{1., 1., 0.};
    Point c{2., 2., 0.};

    double result = signed_area_2D(a, b, c);

    EXPECT_NEAR(result, 0., kTestEpsilon);
}

TEST_F(ProjectionsTest, signed_area_2DRightTriangle) {
    Point a{0., 0., 0.};
    Point b{3., 0., 0.};
    Point c{3., 4., 0.};

    double result = signed_area_2D(a, b, c);

    EXPECT_NEAR(result, 12., kTestEpsilon);
}

//----------------------------------------------
// AxisAlignedBoundingBox tests
//----------------------------------------------

TEST_F(ProjectionsTest, AABBConstructionHorizontal) {
    Point a{1., 2., 0.};
    Point b{3., 2., 0.};

    auto box = axis_aligned_bounding_box(a, b);

    EXPECT_NEAR(box.min_x, 1. - kEpsilon, kTestEpsilon);
    EXPECT_NEAR(box.max_x, 3. + kEpsilon, kTestEpsilon);
    EXPECT_NEAR(box.min_y, 2. - kEpsilon, kTestEpsilon);
    EXPECT_NEAR(box.max_y, 2. + kEpsilon, kTestEpsilon);
}

TEST_F(ProjectionsTest, AABBConstructionVertical) {
    Point a{2., 1., 0.};
    Point b{2., 3., 0.};

    auto box = axis_aligned_bounding_box(a, b);

    EXPECT_NEAR(box.min_x, 2. - kEpsilon, kTestEpsilon);
    EXPECT_NEAR(box.max_x, 2. + kEpsilon, kTestEpsilon);
    EXPECT_NEAR(box.min_y, 1. - kEpsilon, kTestEpsilon);
    EXPECT_NEAR(box.max_y, 3. + kEpsilon, kTestEpsilon);
}

TEST_F(ProjectionsTest, AABBConstructionDiagonal) {
    Point a{1., 1., 0.};
    Point b{3., 4., 0.};

    auto box = axis_aligned_bounding_box(a, b);

    EXPECT_NEAR(box.min_x, 1. - kEpsilon, kTestEpsilon);
    EXPECT_NEAR(box.max_x, 3. + kEpsilon, kTestEpsilon);
    EXPECT_NEAR(box.min_y, 1. - kEpsilon, kTestEpsilon);
    EXPECT_NEAR(box.max_y, 4. + kEpsilon, kTestEpsilon);
}

TEST_F(ProjectionsTest, AABBConstructionCustomPadding) {
    Point a{1., 1., 0.};
    Point b{3., 3., 0.};
    double pad = 0.5;

    auto box = axis_aligned_bounding_box(a, b, pad);

    EXPECT_NEAR(box.min_x, 0.5, kTestEpsilon);
    EXPECT_NEAR(box.max_x, 3.5, kTestEpsilon);
    EXPECT_NEAR(box.min_y, 0.5, kTestEpsilon);
    EXPECT_NEAR(box.max_y, 3.5, kTestEpsilon);
}

TEST_F(ProjectionsTest, AABBOrderIndependent) {
    Point a{1., 1., 0.};
    Point b{3., 3., 0.};

    auto box1 = axis_aligned_bounding_box(a, b);
    auto box2 = axis_aligned_bounding_box(b, a);

    EXPECT_NEAR(box1.min_x, box2.min_x, kTestEpsilon);
    EXPECT_NEAR(box1.max_x, box2.max_x, kTestEpsilon);
    EXPECT_NEAR(box1.min_y, box2.min_y, kTestEpsilon);
    EXPECT_NEAR(box1.max_y, box2.max_y, kTestEpsilon);
}

//----------------------------------------------
// contains_point_3d() tests
//----------------------------------------------

TEST_F(ProjectionsTest, ContainsPointInside) {
    AxisAlignedBoundingBox box{0., 0., 0., 2., 2., 0.};
    Point point{1., 1., 0.};

    EXPECT_TRUE(contains_point_3d(box, point));
}

TEST_F(ProjectionsTest, ContainsPointOnBoundary) {
    AxisAlignedBoundingBox box{0., 0., 0., 2., 2., 0.};

    EXPECT_TRUE(contains_point_3d(box, Point{0., 1., 0.}));
    EXPECT_TRUE(contains_point_3d(box, Point{2., 1., 0.}));
    EXPECT_TRUE(contains_point_3d(box, Point{1., 0., 0.}));
    EXPECT_TRUE(contains_point_3d(box, Point{1., 2., 0.}));
}

TEST_F(ProjectionsTest, ContainsPointAtCorners) {
    AxisAlignedBoundingBox box{0., 0., 0., 2., 2., 0.};

    EXPECT_TRUE(contains_point_3d(box, Point{0., 0., 0.}));
    EXPECT_TRUE(contains_point_3d(box, Point{2., 0., 0.}));
    EXPECT_TRUE(contains_point_3d(box, Point{0., 2., 0.}));
    EXPECT_TRUE(contains_point_3d(box, Point{2., 2., 0.}));
}

TEST_F(ProjectionsTest, ContainsPointOutside) {
    AxisAlignedBoundingBox box{0., 0., 0., 2., 2., 0.};

    EXPECT_FALSE(contains_point_3d(box, Point{-0.1, 1., 0.}));
    EXPECT_FALSE(contains_point_3d(box, Point{2.1, 1., 0.}));
    EXPECT_FALSE(contains_point_3d(box, Point{1., -0.1, 0.}));
    EXPECT_FALSE(contains_point_3d(box, Point{1., 2.1, 0.}));
}

//----------------------------------------------
// on_line_segment_3d() tests
//----------------------------------------------

TEST_F(ProjectionsTest, OnSegmentAtStart) {
    Point a{0., 0., 0.};
    Point b{2., 0., 0.};
    Point p{0., 0., 0.};

    EXPECT_TRUE(on_line_segment_3d(a, b, p));
}

TEST_F(ProjectionsTest, OnSegmentAtEnd) {
    Point a{0., 0., 0.};
    Point b{2., 0., 0.};
    Point p{2., 0., 0.};

    EXPECT_TRUE(on_line_segment_3d(a, b, p));
}

TEST_F(ProjectionsTest, OnSegmentInMiddle) {
    Point a{0., 0., 0.};
    Point b{2., 0., 0.};
    Point p{1., 0., 0.};

    EXPECT_TRUE(on_line_segment_3d(a, b, p));
}

TEST_F(ProjectionsTest, OnSegmentNotCollinear) {
    Point a{0., 0., 0.};
    Point b{2., 0., 0.};
    Point p{1., 1., 0.};

    EXPECT_FALSE(on_line_segment_3d(a, b, p));
}

TEST_F(ProjectionsTest, OnSegmentCollinearButBeyond) {
    Point a{0., 0., 0.};
    Point b{2., 0., 0.};
    Point p{3., 0., 0.};

    EXPECT_FALSE(on_line_segment_3d(a, b, p));
}

TEST_F(ProjectionsTest, OnSegmentDiagonal) {
    Point a{0., 0., 0.};
    Point b{2., 2., 0.};
    Point p{1., 1., 0.};

    EXPECT_TRUE(on_line_segment_3d(a, b, p));
}

//----------------------------------------------
// line_segments_intersect_2d() tests
//----------------------------------------------

TEST_F(ProjectionsTest, SegmentsIntersectProperCrossing) {
    Point a1{0., 0., 0.};
    Point a2{2., 2., 0.};
    Point b1{0., 2., 0.};
    Point b2{2., 0., 0.};

    EXPECT_TRUE(line_segments_intersect_2d(a1, a2, b1, b2));
}

TEST_F(ProjectionsTest, SegmentsIntersectAtEndpoint) {
    Point a1{0., 0., 0.};
    Point a2{1., 1., 0.};
    Point b1{1., 1., 0.};
    Point b2{2., 0., 0.};

    EXPECT_TRUE(line_segments_intersect_2d(a1, a2, b1, b2));
}

TEST_F(ProjectionsTest, SegmentsIntersectTShape) {
    Point a1{0., 1., 0.};
    Point a2{2., 1., 0.};
    Point b1{1., 0., 0.};
    Point b2{1., 2., 0.};

    EXPECT_TRUE(line_segments_intersect_2d(a1, a2, b1, b2));
}

TEST_F(ProjectionsTest, SegmentsParallelNoIntersection) {
    Point a1{0., 0., 0.};
    Point a2{2., 0., 0.};
    Point b1{0., 1., 0.};
    Point b2{2., 1., 0.};

    EXPECT_FALSE(line_segments_intersect_2d(a1, a2, b1, b2));
}

TEST_F(ProjectionsTest, SegmentsCollinearOverlapping) {
    Point a1{0., 0., 0.};
    Point a2{2., 0., 0.};
    Point b1{1., 0., 0.};
    Point b2{3., 0., 0.};

    EXPECT_TRUE(line_segments_intersect_2d(a1, a2, b1, b2));
}

TEST_F(ProjectionsTest, SegmentsCollinearNonOverlapping) {
    Point a1{0., 0., 0.};
    Point a2{1., 0., 0.};
    Point b1{2., 0., 0.};
    Point b2{3., 0., 0.};

    EXPECT_FALSE(line_segments_intersect_2d(a1, a2, b1, b2));
}

TEST_F(ProjectionsTest, SegmentsNoIntersectionSeparated) {
    Point a1{0., 0., 0.};
    Point a2{1., 0., 0.};
    Point b1{2., 2., 0.};
    Point b2{3., 2., 0.};

    EXPECT_FALSE(line_segments_intersect_2d(a1, a2, b1, b2));
}

TEST_F(ProjectionsTest, SegmentsSameSegment) {
    Point a1{0., 0., 0.};
    Point a2{1., 1., 0.};

    EXPECT_TRUE(line_segments_intersect_2d(a1, a2, a1, a2));
}
