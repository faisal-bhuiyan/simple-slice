#include <cmath>

#include <gtest/gtest.h>

#include "geometry/projections.hpp"
#include "geometry/utilities.hpp"
#include "geometry/vector.hpp"

using namespace simple_slice::geometry;

// Test fixture
class ProjectionsTest : public ::testing::Test {
protected:
    static constexpr double kTestEpsilon = 1e-9;

    void ExpectVectorNear(const Vector3D& actual, const Vector3D& expected) {
        EXPECT_NEAR(actual.x, expected.x, kTestEpsilon);
        EXPECT_NEAR(actual.y, expected.y, kTestEpsilon);
        EXPECT_NEAR(actual.z, expected.z, kTestEpsilon);
    }
};

// ----------------------------------------------
// project_point_on_line() tests
// ----------------------------------------------

TEST_F(ProjectionsTest, ProjectPointOnLineOrthogonal) {
    // Point above horizontal line
    Vector3D point{1., 1.};
    Vector3D a{0., 0.};
    Vector3D b{2., 0.};

    Vector3D result = project_point_on_line(point, a, b);

    ExpectVectorNear(result, Vector3D{1., 0.});
}

TEST_F(ProjectionsTest, ProjectPointOnLineAlreadyOnLine) {
    Vector3D point{1., 0.};
    Vector3D a{0., 0.};
    Vector3D b{2., 0.};

    Vector3D result = project_point_on_line(point, a, b);

    ExpectVectorNear(result, point);
}

TEST_F(ProjectionsTest, ProjectPointOnLineBeyondSegment) {
    // Projection beyond endpoint b
    Vector3D point{3., 1.};
    Vector3D a{0., 0.};
    Vector3D b{2., 0.};

    Vector3D result = project_point_on_line(point, a, b);

    ExpectVectorNear(result, Vector3D{3., 0.});  // Beyond b
}

TEST_F(ProjectionsTest, ProjectPointOnLineBeforeSegment) {
    // Projection before endpoint a
    Vector3D point{-1., 1.};
    Vector3D a{0., 0.};
    Vector3D b{2., 0.};

    Vector3D result = project_point_on_line(point, a, b);

    ExpectVectorNear(result, Vector3D{-1., 0.});  // Before a
}

TEST_F(ProjectionsTest, ProjectPointOnLineDiagonal) {
    // 45-degree line
    Vector3D point{2., 0.};
    Vector3D a{0., 0.};
    Vector3D b{1., 1.};

    Vector3D result = project_point_on_line(point, a, b);

    ExpectVectorNear(result, Vector3D{1., 1.});
}

TEST_F(ProjectionsTest, ProjectPointOnLineDegenerateCase) {
    // Line is a point
    Vector3D point{1., 1.};
    Vector3D a{0.5, 0.5};
    Vector3D b{0.5, 0.5};

    Vector3D result = project_point_on_line(point, a, b);

    ExpectVectorNear(result, a);  // Returns a
}

// ----------------------------------------------
// project_point_on_line_segment() tests
// ----------------------------------------------

TEST_F(ProjectionsTest, ProjectPointOnSegmentOrthogonal) {
    Vector3D point{1., 1.};
    Vector3D a{0., 0.};
    Vector3D b{2., 0.};

    Vector3D result = project_point_on_line_segment(point, a, b);

    ExpectVectorNear(result, Vector3D{1., 0.});
}

TEST_F(ProjectionsTest, ProjectPointOnSegmentClampedToStart) {
    Vector3D point{-1., 1.};
    Vector3D a{0., 0.};
    Vector3D b{2., 0.};

    Vector3D result = project_point_on_line_segment(point, a, b);

    ExpectVectorNear(result, a);  // Clamped to start
}

TEST_F(ProjectionsTest, ProjectPointOnSegmentClampedToEnd) {
    Vector3D point{3., 1.};
    Vector3D a{0., 0.};
    Vector3D b{2., 0.};

    Vector3D result = project_point_on_line_segment(point, a, b);

    ExpectVectorNear(result, b);  // Clamped to end
}

TEST_F(ProjectionsTest, ProjectPointOnSegmentAtStart) {
    Vector3D point{0., 1.};
    Vector3D a{0., 0.};
    Vector3D b{2., 0.};

    Vector3D result = project_point_on_line_segment(point, a, b);

    ExpectVectorNear(result, a);
}

TEST_F(ProjectionsTest, ProjectPointOnSegmentAtEnd) {
    Vector3D point{2., 1.};
    Vector3D a{0., 0.};
    Vector3D b{2., 0.};

    Vector3D result = project_point_on_line_segment(point, a, b);

    ExpectVectorNear(result, b);
}

TEST_F(ProjectionsTest, ProjectPointOnSegmentDegenerateCase) {
    Vector3D point{1., 1.};
    Vector3D a{0.5, 0.5};
    Vector3D b{0.5, 0.5};

    Vector3D result = project_point_on_line_segment(point, a, b);

    ExpectVectorNear(result, a);
}

// ----------------------------------------------
// orient2D() tests
// ----------------------------------------------

TEST_F(ProjectionsTest, Orient2DCounterclockwise) {
    Vector3D a{0., 0.};
    Vector3D b{1., 0.};
    Vector3D c{0.5, 1.};  // Above line ab

    double result = orient2D(a, b, c);

    EXPECT_GT(result, 0.);  // CCW is positive
}

TEST_F(ProjectionsTest, Orient2DClockwise) {
    Vector3D a{0., 0.};
    Vector3D b{1., 0.};
    Vector3D c{0.5, -1.};  // Below line ab

    double result = orient2D(a, b, c);

    EXPECT_LT(result, 0.);  // CW is negative
}

TEST_F(ProjectionsTest, Orient2DCollinear) {
    Vector3D a{0., 0.};
    Vector3D b{1., 1.};
    Vector3D c{2., 2.};  // On line ab

    double result = orient2D(a, b, c);

    EXPECT_NEAR(result, 0., kTestEpsilon);
}

TEST_F(ProjectionsTest, Orient2DRightTriangle) {
    Vector3D a{0., 0.};
    Vector3D b{3., 0.};
    Vector3D c{3., 4.};

    double result = orient2D(a, b, c);

    // Twice the triangle area = 3 * 4 = 12
    EXPECT_NEAR(result, 12., kTestEpsilon);
}

// ----------------------------------------------
// AxisAlignedBoundingBox2D tests
// ----------------------------------------------

TEST_F(ProjectionsTest, AABBConstructionHorizontal) {
    Vector3D a{1., 2.};
    Vector3D b{3., 2.};

    auto box = axis_aligned_bounding_box_2d(a, b);

    EXPECT_NEAR(box.min_x, 1. - kEpsilon, kTestEpsilon);
    EXPECT_NEAR(box.max_x, 3. + kEpsilon, kTestEpsilon);
    EXPECT_NEAR(box.min_y, 2. - kEpsilon, kTestEpsilon);
    EXPECT_NEAR(box.max_y, 2. + kEpsilon, kTestEpsilon);
}

TEST_F(ProjectionsTest, AABBConstructionVertical) {
    Vector3D a{2., 1.};
    Vector3D b{2., 3.};

    auto box = axis_aligned_bounding_box_2d(a, b);

    EXPECT_NEAR(box.min_x, 2. - kEpsilon, kTestEpsilon);
    EXPECT_NEAR(box.max_x, 2. + kEpsilon, kTestEpsilon);
    EXPECT_NEAR(box.min_y, 1. - kEpsilon, kTestEpsilon);
    EXPECT_NEAR(box.max_y, 3. + kEpsilon, kTestEpsilon);
}

TEST_F(ProjectionsTest, AABBConstructionDiagonal) {
    Vector3D a{1., 1.};
    Vector3D b{3., 4.};

    auto box = axis_aligned_bounding_box_2d(a, b);

    EXPECT_NEAR(box.min_x, 1. - kEpsilon, kTestEpsilon);
    EXPECT_NEAR(box.max_x, 3. + kEpsilon, kTestEpsilon);
    EXPECT_NEAR(box.min_y, 1. - kEpsilon, kTestEpsilon);
    EXPECT_NEAR(box.max_y, 4. + kEpsilon, kTestEpsilon);
}

TEST_F(ProjectionsTest, AABBConstructionCustomPadding) {
    Vector3D a{1., 1.};
    Vector3D b{3., 3.};
    double pad = 0.5;

    auto box = axis_aligned_bounding_box_2d(a, b, pad);

    EXPECT_NEAR(box.min_x, 0.5, kTestEpsilon);
    EXPECT_NEAR(box.max_x, 3.5, kTestEpsilon);
    EXPECT_NEAR(box.min_y, 0.5, kTestEpsilon);
    EXPECT_NEAR(box.max_y, 3.5, kTestEpsilon);
}

TEST_F(ProjectionsTest, AABBOrderIndependent) {
    Vector3D a{1., 1.};
    Vector3D b{3., 3.};

    auto box1 = axis_aligned_bounding_box_2d(a, b);
    auto box2 = axis_aligned_bounding_box_2d(b, a);  // Reversed

    EXPECT_NEAR(box1.min_x, box2.min_x, kTestEpsilon);
    EXPECT_NEAR(box1.max_x, box2.max_x, kTestEpsilon);
    EXPECT_NEAR(box1.min_y, box2.min_y, kTestEpsilon);
    EXPECT_NEAR(box1.max_y, box2.max_y, kTestEpsilon);
}

// ----------------------------------------------
// contains_point_2d() tests
// ----------------------------------------------

TEST_F(ProjectionsTest, ContainsPointInside) {
    AxisAlignedBoundingBox2D box{0., 0., 2., 2.};
    Vector3D point{1., 1.};

    EXPECT_TRUE(contains_point_2d(box, point));
}

TEST_F(ProjectionsTest, ContainsPointOnBoundary) {
    AxisAlignedBoundingBox2D box{0., 0., 2., 2.};

    EXPECT_TRUE(contains_point_2d(box, Vector3D{0., 1.}));  // Left edge
    EXPECT_TRUE(contains_point_2d(box, Vector3D{2., 1.}));  // Right edge
    EXPECT_TRUE(contains_point_2d(box, Vector3D{1., 0.}));  // Bottom edge
    EXPECT_TRUE(contains_point_2d(box, Vector3D{1., 2.}));  // Top edge
}

TEST_F(ProjectionsTest, ContainsPointAtCorners) {
    AxisAlignedBoundingBox2D box{0., 0., 2., 2.};

    EXPECT_TRUE(contains_point_2d(box, Vector3D{0., 0.}));
    EXPECT_TRUE(contains_point_2d(box, Vector3D{2., 0.}));
    EXPECT_TRUE(contains_point_2d(box, Vector3D{0., 2.}));
    EXPECT_TRUE(contains_point_2d(box, Vector3D{2., 2.}));
}

TEST_F(ProjectionsTest, ContainsPointOutside) {
    AxisAlignedBoundingBox2D box{0., 0., 2., 2.};

    EXPECT_FALSE(contains_point_2d(box, Vector3D{-0.1, 1.}));  // Left
    EXPECT_FALSE(contains_point_2d(box, Vector3D{2.1, 1.}));   // Right
    EXPECT_FALSE(contains_point_2d(box, Vector3D{1., -0.1}));  // Below
    EXPECT_FALSE(contains_point_2d(box, Vector3D{1., 2.1}));   // Above
    EXPECT_FALSE(contains_point_2d(box, Vector3D{-1., -1.}));  // Corner
}

// ----------------------------------------------
// on_line_segment_2d() tests
// ----------------------------------------------

TEST_F(ProjectionsTest, OnSegmentAtStart) {
    Vector3D a{0., 0.};
    Vector3D b{2., 0.};
    Vector3D point{0., 0.};

    EXPECT_TRUE(on_line_segment_2d(a, b, point));
}

TEST_F(ProjectionsTest, OnSegmentAtEnd) {
    Vector3D a{0., 0.};
    Vector3D b{2., 0.};
    Vector3D point{2., 0.};

    EXPECT_TRUE(on_line_segment_2d(a, b, point));
}

TEST_F(ProjectionsTest, OnSegmentInMiddle) {
    Vector3D a{0., 0.};
    Vector3D b{2., 0.};
    Vector3D point{1., 0.};

    EXPECT_TRUE(on_line_segment_2d(a, b, point));
}

TEST_F(ProjectionsTest, OnSegmentNotCollinear) {
    Vector3D a{0., 0.};
    Vector3D b{2., 0.};
    Vector3D point{1., 1.};  // Above the segment

    EXPECT_FALSE(on_line_segment_2d(a, b, point));
}

TEST_F(ProjectionsTest, OnSegmentCollinearButBeyond) {
    Vector3D a{0., 0.};
    Vector3D b{2., 0.};
    Vector3D point{3., 0.};  // Collinear but beyond b

    EXPECT_FALSE(on_line_segment_2d(a, b, point));
}

TEST_F(ProjectionsTest, OnSegmentDiagonal) {
    Vector3D a{0., 0.};
    Vector3D b{2., 2.};
    Vector3D point{1., 1.};

    EXPECT_TRUE(on_line_segment_2d(a, b, point));
}

// ----------------------------------------------
// line_segments_intersect_2d() tests
// ----------------------------------------------

TEST_F(ProjectionsTest, SegmentsIntersectProperCrossing) {
    Vector3D a1{0., 0.};
    Vector3D a2{2., 2.};
    Vector3D b1{0., 2.};
    Vector3D b2{2., 0.};

    EXPECT_TRUE(line_segments_intersect_2d(a1, a2, b1, b2));
}

TEST_F(ProjectionsTest, SegmentsIntersectAtEndpoint) {
    Vector3D a1{0., 0.};
    Vector3D a2{1., 1.};
    Vector3D b1{1., 1.};
    Vector3D b2{2., 0.};

    EXPECT_TRUE(line_segments_intersect_2d(a1, a2, b1, b2));
}

TEST_F(ProjectionsTest, SegmentsIntersectTShape) {
    Vector3D a1{0., 1.};
    Vector3D a2{2., 1.};
    Vector3D b1{1., 0.};
    Vector3D b2{1., 2.};

    EXPECT_TRUE(line_segments_intersect_2d(a1, a2, b1, b2));
}

TEST_F(ProjectionsTest, SegmentsParallelNoIntersection) {
    Vector3D a1{0., 0.};
    Vector3D a2{2., 0.};
    Vector3D b1{0., 1.};
    Vector3D b2{2., 1.};

    EXPECT_FALSE(line_segments_intersect_2d(a1, a2, b1, b2));
}

TEST_F(ProjectionsTest, SegmentsCollinearOverlapping) {
    Vector3D a1{0., 0.};
    Vector3D a2{2., 0.};
    Vector3D b1{1., 0.};
    Vector3D b2{3., 0.};

    EXPECT_TRUE(line_segments_intersect_2d(a1, a2, b1, b2));
}

TEST_F(ProjectionsTest, SegmentsCollinearNonOverlapping) {
    Vector3D a1{0., 0.};
    Vector3D a2{1., 0.};
    Vector3D b1{2., 0.};
    Vector3D b2{3., 0.};

    EXPECT_FALSE(line_segments_intersect_2d(a1, a2, b1, b2));
}

TEST_F(ProjectionsTest, SegmentsNoIntersectionSeparated) {
    Vector3D a1{0., 0.};
    Vector3D a2{1., 0.};
    Vector3D b1{2., 2.};
    Vector3D b2{3., 2.};

    EXPECT_FALSE(line_segments_intersect_2d(a1, a2, b1, b2));
}

TEST_F(ProjectionsTest, SegmentsSameSegment) {
    Vector3D a1{0., 0.};
    Vector3D a2{1., 1.};

    EXPECT_TRUE(line_segments_intersect_2d(a1, a2, a1, a2));
}
