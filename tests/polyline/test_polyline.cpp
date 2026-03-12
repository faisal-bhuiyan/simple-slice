#include <iostream>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "polyline/polyline.hpp"

using simple_slice::polyline::Point;
using simple_slice::polyline::Polyline;
using simple_slice::polyline::PolylineRepresentation;
using simple_slice::polyline::PolylineType;
using simple_slice::polyline::VertexIndex;

//----------------------------------------------------------------------------------
// Part (A): Compressed Vertex Ordering — Determinism
//----------------------------------------------------------------------------------
// Requirement: permuting the order of segments in the 2N buffer, and/or
// flipping the two vertex indices within a segment, must yield the SAME
// compressed ordering (starting at the endpoint with smaller index).
//----------------------------------------------------------------------------------

TEST(PartA_Determinism, CanonicalOrder) {
    //  0 --- 1 --- 2 --- 3
    // Segments in natural order: (0,1), (1,2), (2,3)
    const std::vector<VertexIndex> segments = {0, 1, 1, 2, 2, 3};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{0, 1, 2, 3}));
}

TEST(PartA_Determinism, PermutedSegmentOrder) {
    // Same segments, different order: (2,3), (0,1), (1,2)
    const std::vector<VertexIndex> segments = {2, 3, 0, 1, 1, 2};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{0, 1, 2, 3}));
}

TEST(PartA_Determinism, FlippedVerticesWithinSegments) {
    // Each segment's vertex pair is flipped: (1,0), (2,1), (3,2)
    const std::vector<VertexIndex> segments = {1, 0, 2, 1, 3, 2};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{0, 1, 2, 3}));
}

TEST(PartA_Determinism, PermutedAndFlipped) {
    // Both permuted and flipped: (3,2), (1,0), (2,1)
    const std::vector<VertexIndex> segments = {3, 2, 1, 0, 2, 1};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{0, 1, 2, 3}));
}

TEST(PartA_Determinism, AnotherMixedVariant) {
    // Another permutation + partial flip: (1,2), (3,2), (0,1)
    const std::vector<VertexIndex> segments = {1, 2, 3, 2, 0, 1};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{0, 1, 2, 3}));
}

TEST(PartA_Determinism, ReverseCanonical) {
    // Segments listed in reverse: (2,3), (1,2), (0,1)
    const std::vector<VertexIndex> segments = {2, 3, 1, 2, 0, 1};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{0, 1, 2, 3}));
}

TEST(PartA_Determinism, ExamFigure3Example) {
    //  0 --- 1 --- 2 --- 3
    //
    // Exam Figure 3: segments given as [0, 1, 3, 2, 1, 2]
    //   seg 0: (0,1)   seg 1: (3,2)   seg 2: (1,2)
    // After compression: [0, 1, 2, 3]
    const std::vector<VertexIndex> segments = {0, 1, 3, 2, 1, 2};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{0, 1, 2, 3}));
}

//----------------------------------------------------------------------------------
// Part (A): Edge Cases
//----------------------------------------------------------------------------------

TEST(PartA_EdgeCases, SingleSegment) {
    //  1 --- 3
    // Minimal polyline: one segment connecting vertices 3 and 1
    // Start at the smaller endpoint -> [1, 3]
    const std::vector<VertexIndex> segments = {3, 1};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{1, 3}));
}

TEST(PartA_EdgeCases, SingleSegmentFlipped) {
    // Same segment flipped: (1, 3) instead of (3, 1)
    const std::vector<VertexIndex> segments = {1, 3};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{1, 3}));
}

TEST(PartA_EdgeCases, TwoSegments) {
    //  0 --- 2 --- 1
    // Start at smaller endpoint (0), walk to 2, then to 1 -> [0, 2, 1]
    const std::vector<VertexIndex> segments = {2, 0, 2, 1};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{0, 2, 1}));
}

TEST(PartA_EdgeCases, TwoSegmentsPermuted) {
    // Same two segments permuted: (2,1), (2,0)
    const std::vector<VertexIndex> segments = {2, 1, 2, 0};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{0, 2, 1}));
}

TEST(PartA_EdgeCases, LongerChainFiveVertices) {
    //  0 --- 1 --- 2 --- 3 --- 4
    // Scrambled segments: (3,4), (0,1), (2,1), (3,2)
    const std::vector<VertexIndex> segments = {3, 4, 0, 1, 2, 1, 3, 2};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{0, 1, 2, 3, 4}));
}

TEST(PartA_EdgeCases, NonContiguousVertexIndices) {
    //  1 --- 3 --- 5       (vertices 0, 2, 4 not used)
    // Segments: (5,3), (1,3) -> permuted and partially flipped
    const std::vector<VertexIndex> segments = {5, 3, 1, 3};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{1, 3, 5}));
}

//----------------------------------------------------------------------------------
// Part (A): Polygon Compressed Ordering
//----------------------------------------------------------------------------------
// For polygons, all vertices have degree 2.  The walk starts at the smallest
// participating vertex and follows the first neighbor found.

TEST(PartA_Polygon, ClosedTriangle) {
    /*
     *      0
     *     / \
     *    2---1
     */
    const std::vector<VertexIndex> segments = {0, 1, 1, 2, 2, 0};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    const auto& ordering = p.GetCompressedSegments();
    // Must start and end at the same vertex
    EXPECT_EQ(ordering.front(), ordering.back());
    // Must have 4 entries (3 unique vertices + closing repeat)
    EXPECT_EQ(ordering.size(), 4u);
    // Must start at vertex 0 (smallest)
    EXPECT_EQ(ordering.front(), 0u);
}

TEST(PartA_Polygon, ClosedQuad) {
    /*
     *  0 --- 1
     *  |     |
     *  3 --- 2
     */
    const std::vector<VertexIndex> segments = {0, 1, 1, 2, 2, 3, 3, 0};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    const auto& ordering = p.GetCompressedSegments();
    EXPECT_EQ(ordering.front(), ordering.back());
    EXPECT_EQ(ordering.size(), 5u);
    EXPECT_EQ(ordering.front(), 0u);
}

TEST(PartA_Polygon, ClosedQuadPermutedAndFlipped) {
    // Same quadrilateral with scrambled segments: (3,0), (2,1), (0,1), (3,2)
    const std::vector<VertexIndex> segments = {3, 0, 2, 1, 0, 1, 3, 2};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    const auto& ordering = p.GetCompressedSegments();
    EXPECT_EQ(ordering.front(), ordering.back());
    EXPECT_EQ(ordering.size(), 5u);
    EXPECT_EQ(ordering.front(), 0u);
}

TEST(PartA_Polygon, ClosedTrianglePermutedAndFlipped) {
    // Triangle scrambled: (2,0), (1,0), (2,1)
    const std::vector<VertexIndex> segments = {2, 0, 1, 0, 2, 1};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    const auto& ordering = p.GetCompressedSegments();
    EXPECT_EQ(ordering.front(), ordering.back());
    EXPECT_EQ(ordering.size(), 4u);
    EXPECT_EQ(ordering.front(), 0u);
}

//----------------------------------------------------------------------------------
// Part (B): Polygon vs Polyline Detection — IsPolygon()
//----------------------------------------------------------------------------------
// Requirement: IsPolygon() returns true for closed polygons and false for
// open polylines.  Tested via both raw segment and compressed ordering input.
//----------------------------------------------------------------------------------

TEST(PartB_IsPolygon, OpenPolylineFromSegments) {
    //  0 --- 1 --- 2 --- 3    (endpoints at 0 and 3)
    const std::vector<VertexIndex> segments = {0, 1, 1, 2, 2, 3};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_FALSE(p.IsPolygon());
    EXPECT_EQ(p.GetType(), PolylineType::kOpen);
}

TEST(PartB_IsPolygon, ClosedPolygonFromSegments) {
    //  0 --- 1
    //  |     |    (all degree 2)
    //  3 --- 2
    const std::vector<VertexIndex> segments = {0, 1, 1, 2, 2, 3, 3, 0};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_TRUE(p.IsPolygon());
    EXPECT_EQ(p.GetType(), PolylineType::kClosed);
}

TEST(PartB_IsPolygon, ClosedTriangleFromSegments) {
    const std::vector<VertexIndex> segments = {0, 1, 1, 2, 2, 0};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_TRUE(p.IsPolygon());
    EXPECT_EQ(p.GetType(), PolylineType::kClosed);
}

TEST(PartB_IsPolygon, SingleSegmentIsOpen) {
    const std::vector<VertexIndex> segments = {0, 1};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_FALSE(p.IsPolygon());
    EXPECT_EQ(p.GetType(), PolylineType::kOpen);
}

TEST(PartB_IsPolygon, OpenPolylineFromCompressedOrdering) {
    // Compressed ordering for open polyline: [0, 1, 2, 3] — front != back
    const std::vector<VertexIndex> ordering = {0, 1, 2, 3};
    Polyline p(PolylineRepresentation::kCompressedVertexOrdering, ordering);
    EXPECT_FALSE(p.IsPolygon());
    EXPECT_EQ(p.GetType(), PolylineType::kOpen);
}

TEST(PartB_IsPolygon, ClosedPolygonFromCompressedOrdering) {
    // Compressed ordering for polygon: [0, 1, 2, 3, 0] — front == back
    const std::vector<VertexIndex> ordering = {0, 1, 2, 3, 0};
    Polyline p(PolylineRepresentation::kCompressedVertexOrdering, ordering);
    EXPECT_TRUE(p.IsPolygon());
    EXPECT_EQ(p.GetType(), PolylineType::kClosed);
}

TEST(PartB_IsPolygon, ClosedTriangleFromCompressedOrdering) {
    const std::vector<VertexIndex> ordering = {0, 1, 2, 0};
    Polyline p(PolylineRepresentation::kCompressedVertexOrdering, ordering);
    EXPECT_TRUE(p.IsPolygon());
    EXPECT_EQ(p.GetType(), PolylineType::kClosed);
}

TEST(PartB_IsPolygon, TwoVertexOpenFromCompressedOrdering) {
    const std::vector<VertexIndex> ordering = {1, 3};
    Polyline p(PolylineRepresentation::kCompressedVertexOrdering, ordering);
    EXPECT_FALSE(p.IsPolygon());
    EXPECT_EQ(p.GetType(), PolylineType::kOpen);
}

//----------------------------------------------------------------------------------
// Part (B): Roundtrip — compress then detect
//----------------------------------------------------------------------------------
// Verify that constructing from raw segments and then checking IsPolygon()
// gives the same answer as constructing from the resulting compressed ordering.

TEST(PartB_Roundtrip, OpenPolyline) {
    const std::vector<VertexIndex> segments = {3, 2, 1, 0, 2, 1};
    Polyline from_raw(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_FALSE(from_raw.IsPolygon());

    Polyline from_compressed(
        PolylineRepresentation::kCompressedVertexOrdering, from_raw.GetCompressedSegments()
    );
    EXPECT_FALSE(from_compressed.IsPolygon());
    EXPECT_EQ(from_raw.GetCompressedSegments(), from_compressed.GetCompressedSegments());
}

TEST(PartB_Roundtrip, ClosedPolygon) {
    const std::vector<VertexIndex> segments = {3, 0, 2, 1, 0, 1, 3, 2};
    Polyline from_raw(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_TRUE(from_raw.IsPolygon());

    Polyline from_compressed(
        PolylineRepresentation::kCompressedVertexOrdering, from_raw.GetCompressedSegments()
    );
    EXPECT_TRUE(from_compressed.IsPolygon());
    EXPECT_EQ(from_raw.GetCompressedSegments(), from_compressed.GetCompressedSegments());
}

//----------------------------------------------------------------------------------
// Part (A+B): GetType() consistency
//----------------------------------------------------------------------------------
// Verify that GetType() agrees with IsPolygon() for every construction path

TEST(TypeConsistency, OpenPolylineType) {
    const std::vector<VertexIndex> segments = {0, 1, 1, 2};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetType(), PolylineType::kOpen);
    EXPECT_FALSE(p.IsPolygon());
}

TEST(TypeConsistency, ClosedPolygonType) {
    const std::vector<VertexIndex> segments = {0, 1, 1, 2, 2, 0};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetType(), PolylineType::kClosed);
    EXPECT_TRUE(p.IsPolygon());
}

TEST(TypeConsistency, CompressedOpenType) {
    Polyline p(PolylineRepresentation::kCompressedVertexOrdering, {0, 1, 2});
    EXPECT_EQ(p.GetType(), PolylineType::kOpen);
}

TEST(TypeConsistency, CompressedClosedType) {
    Polyline p(PolylineRepresentation::kCompressedVertexOrdering, {0, 1, 2, 0});
    EXPECT_EQ(p.GetType(), PolylineType::kClosed);
}

//----------------------------------------------------------------------------------
// Input Validation — Constructor Guards
//----------------------------------------------------------------------------------
// The constructor validates the problem's assumptions:
//   - data must not be empty
//   - raw segments must have an even number of entries
//   - every vertex must have degree <= 2
//   - the number of degree-1 endpoints must be 0 or 2

TEST(InputValidation, EmptyDataThrows) {
    EXPECT_THROW(Polyline(PolylineRepresentation::kVerboseSegments, {}), std::invalid_argument);
}

TEST(InputValidation, EmptyCompressedDataThrows) {
    EXPECT_THROW(
        Polyline(PolylineRepresentation::kCompressedVertexOrdering, {}), std::invalid_argument
    );
}

TEST(InputValidation, OddSegmentCountThrows) {
    // 3 entries is not a valid 2N buffer
    EXPECT_THROW(
        Polyline(PolylineRepresentation::kVerboseSegments, {0, 1, 2}), std::invalid_argument
    );
}

TEST(InputValidation, VertexDegreeGreaterThan2Throws) {
    //        0
    //        |             vertex 1 has degree 3 -> invalid
    //  2 --- 1 --- 3
    const std::vector<VertexIndex> segments = {0, 1, 1, 2, 1, 3};
    EXPECT_THROW(
        Polyline(PolylineRepresentation::kVerboseSegments, segments), std::invalid_argument
    );
}

TEST(InputValidation, FourEndpointsThrows) {
    //  0 --- 1     2 --- 3   (disconnected: four degree-1 vertices)
    const std::vector<VertexIndex> segments = {0, 1, 2, 3};
    EXPECT_THROW(
        Polyline(PolylineRepresentation::kVerboseSegments, segments), std::invalid_argument
    );
}

//----------------------------------------------------------------------------------
// Vertices constructor — verify vertices are preserved when provided
//----------------------------------------------------------------------------------

TEST(VerticesConstructor, VerticesStored) {
    const std::vector<Point> vertices = {
        {0., 0., 0.}, {1., 0., 0.}, {2., 0., 0.}, {3., 0., 0.},
        {4., 0., 0.}, {5., 0., 0.}, {6., 0., 0.}, {7., 0., 0.},
    };
    const std::vector<VertexIndex> segments = {0, 1, 1, 2};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments, vertices);
    EXPECT_EQ(p.GetVertices(), vertices);
}

TEST(VerticesConstructor, SameBehaviorAsWithout) {
    const std::vector<Point> vertices = {
        {0., 0., 0.}, {1., 0., 0.}, {2., 0., 0.}, {3., 0., 0.},
        {4., 0., 0.}, {5., 0., 0.}, {6., 0., 0.}, {7., 0., 0.},
    };
    const std::vector<VertexIndex> segments = {0, 1, 1, 2, 2, 3};
    Polyline with_verts(PolylineRepresentation::kVerboseSegments, segments, vertices);
    Polyline without_verts(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(with_verts.GetCompressedSegments(), without_verts.GetCompressedSegments());
    EXPECT_EQ(with_verts.GetType(), without_verts.GetType());
    EXPECT_EQ(with_verts.IsPolygon(), without_verts.IsPolygon());
}

//----------------------------------------------------------------------------------
// Sparse Vertex Indices — not all vertex indices between 0 and max are used
//----------------------------------------------------------------------------------
// The segments buffer may reference non-contiguous vertex indices, leaving
// gaps (degree-0 vertices) in the index space.  The algorithm must skip
// these unused vertices and still produce the correct compressed ordering.
//----------------------------------------------------------------------------------

TEST(SparseVertices, GapInMiddleOpenPolyline) {
    //  0 --- 1 --- 3       (vertex 2 unused)
    // Segments: (0,1), (1,3)
    const std::vector<VertexIndex> segments = {0, 1, 1, 3};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{0, 1, 3}));
    EXPECT_FALSE(p.IsPolygon());
}

TEST(SparseVertices, GapInMiddlePermuted) {
    // Same polyline, segments permuted and flipped: (3,1), (1,0)
    const std::vector<VertexIndex> segments = {3, 1, 1, 0};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{0, 1, 3}));
}

TEST(SparseVertices, LargeGapBetweenIndices) {
    //  0 --- 1 --- 100     (vertices 2..99 unused)
    const std::vector<VertexIndex> segments = {100, 1, 0, 1};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{0, 1, 100}));
    EXPECT_FALSE(p.IsPolygon());
}

TEST(SparseVertices, OnlyHighIndicesUsed) {
    //  5 --- 7 --- 9       (vertices 0..4, 6, 8 unused)
    const std::vector<VertexIndex> segments = {9, 7, 5, 7};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{5, 7, 9}));
    EXPECT_FALSE(p.IsPolygon());
}

TEST(SparseVertices, SingleSegmentWithLargeGap) {
    //  0 --- 5             (vertices 1..4 unused)
    const std::vector<VertexIndex> segments = {0, 5};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{0, 5}));
    EXPECT_FALSE(p.IsPolygon());
}

TEST(SparseVertices, PolygonWithGaps) {
    //      0
    //     / \              (vertices 1 and 3 unused)
    //    4---2
    // Segments: (0,2), (2,4), (4,0)
    const std::vector<VertexIndex> segments = {0, 2, 2, 4, 4, 0};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    const auto& ordering = p.GetCompressedSegments();
    EXPECT_TRUE(p.IsPolygon());
    EXPECT_EQ(ordering.front(), 0u);  // starts at smallest participating vertex
    EXPECT_EQ(ordering.front(), ordering.back());
    EXPECT_EQ(ordering.size(), 4u);  // 3 unique vertices + closing repeat
}

TEST(SparseVertices, PolygonWithGapsPermuted) {
    // Same triangle, scrambled: (4,0), (2,0), (4,2)
    const std::vector<VertexIndex> segments = {4, 0, 2, 0, 4, 2};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    const auto& ordering = p.GetCompressedSegments();
    EXPECT_TRUE(p.IsPolygon());
    EXPECT_EQ(ordering.front(), 0u);
    EXPECT_EQ(ordering.front(), ordering.back());
}

TEST(SparseVertices, LongerChainMultipleGaps) {
    //  1 --- 3 --- 5 --- 7 --- 9   (only odd vertices, all evens unused)
    // Segments scrambled: (7,9), (3,1), (5,3), (7,5)
    const std::vector<VertexIndex> segments = {7, 9, 3, 1, 5, 3, 7, 5};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_EQ(p.GetCompressedSegments(), (std::vector<VertexIndex>{1, 3, 5, 7, 9}));
    EXPECT_FALSE(p.IsPolygon());
}

//----------------------------------------------------------------------------------
// Additional Input Validation — degenerate and duplicate cases
//----------------------------------------------------------------------------------

TEST(InputValidation, DuplicateSegmentCausesDegreeViolation) {
    //  0 === 1 --- 2     (=== means duplicate edge; vertex 1 reaches degree 3)
    const std::vector<VertexIndex> segments = {0, 1, 0, 1, 1, 2};
    EXPECT_THROW(
        Polyline(PolylineRepresentation::kVerboseSegments, segments), std::invalid_argument
    );
}

TEST(InputValidation, SelfLoopSegment) {
    //  ╭─╮
    //  │0│    self-loop: degree 2, endpoint count 0 -> degenerate polygon
    //  ╰─╯
    const std::vector<VertexIndex> segments = {0, 0};
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    EXPECT_TRUE(p.IsPolygon());
}

//----------------------------------------------------------------------------------
// Static Method — GetCompressedVertexOrdering with sparse indices
//----------------------------------------------------------------------------------

TEST(StaticCompression, SparseIndicesToStatic) {
    // Directly call the static method with sparse vertex indices
    // Polyline: 0—2—4, num_vertices = 5 (indices 0..4, with 1 and 3 unused)
    const std::vector<VertexIndex> segments = {2, 0, 4, 2};
    auto ordering = Polyline::GetCompressedVertexOrdering(segments, 5);
    EXPECT_EQ(ordering, (std::vector<VertexIndex>{0, 2, 4}));
}

TEST(StaticCompression, SparsePolygonToStatic) {
    // Triangle 0—2—4—0, num_vertices = 5
    const std::vector<VertexIndex> segments = {0, 2, 2, 4, 4, 0};
    auto ordering = Polyline::GetCompressedVertexOrdering(segments, 5);
    EXPECT_EQ(ordering.front(), 0u);
    EXPECT_EQ(ordering.front(), ordering.back());
    EXPECT_EQ(ordering.size(), 4u);
}

#include <chrono>

//----------------------------------------------------------------------------------
// Performance / Stress Test — long polyline chains
//----------------------------------------------------------------------------------

TEST(Performance, VeryLongOpenPolyline) {
    constexpr VertexIndex num_vertices = 200'000;

    // Build a long chain: 0--1--2--...--(N-1)
    std::vector<VertexIndex> segments;
    segments.reserve(static_cast<size_t>(2 * (num_vertices - 1)));

    for (VertexIndex i = 0; i < num_vertices - 1; ++i) {
        segments.push_back(i);
        segments.push_back(i + 1);
    }

    const auto start = std::chrono::steady_clock::now();

    Polyline p(PolylineRepresentation::kVerboseSegments, segments);

    const auto end = std::chrono::steady_clock::now();
    const auto elapsed_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "[Performance.VeryLongOpenPolyline] elapsed_ms=" << elapsed_ms << "\n";
    // Average time from above: [Performance.VeryLongOpenPolyline] elapsed_ms=30
    // On a system with:
    // Processor	Intel(R) Core(TM) i5-7200U CPU @ 2.50GHz   2.70 GHz
    // Installed RAM	8.00 GB (7.73 GB usable)

    // Correctness checks
    const auto& ordering = p.GetCompressedSegments();
    ASSERT_EQ(ordering.size(), static_cast<size_t>(num_vertices));
    EXPECT_EQ(ordering.front(), 0);
    EXPECT_EQ(ordering.back(), num_vertices - 1);
    EXPECT_FALSE(p.IsPolygon());
}

TEST(Performance, WorstCasePermutation) {
    constexpr VertexIndex num_vertices = 50'000;

    std::vector<VertexIndex> segments;
    segments.reserve(static_cast<size_t>(2 * (num_vertices - 1)));

    for (VertexIndex i = 0; i < num_vertices - 1; ++i) {
        segments.push_back(i);
        segments.push_back(i + 1);
    }

    // Reverse the segments to create a worst-case permutation for polyline determinism algorithm
    std::reverse(segments.begin(), segments.end());

    // Verify that the compressed ordering is correct
    Polyline p(PolylineRepresentation::kVerboseSegments, segments);
    const auto& ordering = p.GetCompressedSegments();
    ASSERT_EQ(ordering.size(), static_cast<size_t>(num_vertices));
    EXPECT_EQ(ordering.front(), 0);
    EXPECT_EQ(ordering.back(), num_vertices - 1);
    EXPECT_FALSE(p.IsPolygon());
}
