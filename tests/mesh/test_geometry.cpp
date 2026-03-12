#include <sstream>
#include <string>
#include <unordered_map>

#include <gtest/gtest.h>

#include "mesh/geometry.hpp"

using simple_slice::mesh::Edge;
using simple_slice::mesh::EdgeEquality;
using simple_slice::mesh::EdgeHash;
using simple_slice::mesh::make_edge;
using simple_slice::mesh::Point;
using simple_slice::mesh::PointEquality;
using simple_slice::mesh::PointHash;
using simple_slice::mesh::Triangle;

//---------------------------------------------------------------------------
// Point hashing and equality
//---------------------------------------------------------------------------

TEST(PointHash, IdenticalPointsHaveSameHash) {
    Point p1{1., 2., 3.};
    Point p2{1., 2., 3.};

    PointHash hash;
    EXPECT_EQ(hash(p1), hash(p2));
}

TEST(PointHash, HandlesZeroAndNegativeCoordinates) {
    Point p1{0., -1., 2.};
    Point p2{0., -1., 2.};

    PointHash hash;
    EXPECT_EQ(hash(p1), hash(p2));
}

TEST(PointHashEqualityContract, EqualPointsHashEqual) {
    Point p1{3.14, 2.71, 1.41};
    Point p2{3.14, 2.71, 1.41};

    PointHash hash;
    PointEquality eq;

    ASSERT_TRUE(eq(p1, p2));
    EXPECT_EQ(hash(p1), hash(p2));
}

TEST(PointEquality, IdenticalPointsAreEqual) {
    Point p1{1., 2., 3.};
    Point p2{1., 2., 3.};

    PointEquality eq;
    EXPECT_TRUE(eq(p1, p2));
}

TEST(PointEquality, DifferentPointsAreNotEqual) {
    Point p1{1., 2., 3.};
    Point p2{1., 2., 4.};

    PointEquality eq;
    EXPECT_FALSE(eq(p1, p2));
}

TEST(PointEquality, NegativeCoordinatesCompareCorrectly) {
    Point p1{-1., -2., -3.};
    Point p2{-1., -2., -3.};

    PointEquality eq;
    EXPECT_TRUE(eq(p1, p2));
}

//---------------------------------------------------------------------------
// Edge canonicalization
//---------------------------------------------------------------------------

TEST(MakeEdge, CanonicalizesOrder) {
    Point p{0., 0., 0.};
    Point q{1., 0., 0.};

    Edge e1 = make_edge(p, q);
    Edge e2 = make_edge(q, p);

    EXPECT_TRUE(PointEquality{}(e1.first, e2.first));
    EXPECT_TRUE(PointEquality{}(e1.second, e2.second));
}

TEST(MakeEdge, LexicographicOrderingIsUsed) {
    Point p{1., 0., 0.};
    Point q{0., 0., 0.};

    Edge e = make_edge(p, q);

    EXPECT_TRUE(PointEquality{}(e.first, q));
    EXPECT_TRUE(PointEquality{}(e.second, p));
}

TEST(MakeEdge, DegenerateEdgeWithIdenticalEndpoints) {
    Point p{1., 1., 1.};

    Edge e = make_edge(p, p);

    EXPECT_TRUE(PointEquality{}(e.first, p));
    EXPECT_TRUE(PointEquality{}(e.second, p));
}

//---------------------------------------------------------------------------
// Edge hashing and equality
//---------------------------------------------------------------------------

TEST(EdgeHash, IdenticalEdgesHaveSameHash) {
    Point p{0., 0., 0.};
    Point q{1., 0., 0.};

    Edge e1 = make_edge(p, q);
    Edge e2 = make_edge(q, p);

    EdgeHash hash;
    EXPECT_EQ(hash(e1), hash(e2));
}

TEST(EdgeEquality, IdenticalEdgesAreEqual) {
    Point p{0., 0., 0.};
    Point q{1., 0., 0.};

    Edge e1 = make_edge(p, q);
    Edge e2 = make_edge(q, p);

    EdgeEquality eq;
    EXPECT_TRUE(eq(e1, e2));
}

TEST(EdgeEquality, DifferentEdgesAreNotEqual) {
    Point p{0., 0., 0.};
    Point q{1., 0., 0.};
    Point r{0., 1., 0.};

    Edge e1 = make_edge(p, q);
    Edge e2 = make_edge(p, r);

    EdgeEquality eq;
    EXPECT_FALSE(eq(e1, e2));
}

TEST(EdgeHash, SymmetricUnderEndpointPermutation) {
    Point a{0., 0., 0.};
    Point b{1., 2., 3.};

    EdgeHash hash;

    EXPECT_EQ(
        hash(make_edge(a, b)), hash(make_edge(b, a))
    );  // hash of edge a -> b should be equal to hash of edge b -> a
}

//---------------------------------------------------------------------------
// Unordered map behavior
//---------------------------------------------------------------------------

TEST(EdgeHash, WorksInUnorderedMap) {
    std::unordered_map<Edge, int, EdgeHash, EdgeEquality> map;

    Point p{0., 0., 0.};
    Point q{1., 0., 0.};

    Edge e1 = make_edge(p, q);  // add edge p -> q
    Edge e2 = make_edge(q, p);  // add edge q -> p

    map[e1] = 42;  // assign value 42 for edge p -> q

    EXPECT_EQ(map.size(), 1u);  // map should contain only one edge: p-> q
    EXPECT_EQ(map[e2], 42);     // value for edge q -> p should be 42
}

TEST(EdgeHash, UnorderedMapOverwriteDoesNotCreateDuplicates) {
    std::unordered_map<Edge, int, EdgeHash, EdgeEquality> map;

    Point a{0., 0., 0.};
    Point b{1., 0., 0.};

    map[make_edge(a, b)] = 1;  // add edge (a, b) with value 1
    map[make_edge(b, a)] = 2;  // add edge (b, a) with value 2

    EXPECT_EQ(map.size(), 1u);          // map should contain only one edge : a -> b
    EXPECT_EQ(map.begin()->second, 2);  // value should be 2 for edge
}

//---------------------------------------------------------------------------
// Triangle structure
//---------------------------------------------------------------------------

TEST(Triangle, StoresVerticesCorrectly) {
    Triangle t{{0., 0., 0.}, {1., 0., 0.}, {0., 1., 0.}};

    EXPECT_EQ(t.a, (Point{0., 0., 0.}));
    EXPECT_EQ(t.b, (Point{1., 0., 0.}));
    EXPECT_EQ(t.c, (Point{0., 1., 0.}));
}

TEST(Triangle, VerticesAreIndependentCopies) {
    Point a{0., 0., 0.};
    Point b{1., 0., 0.};
    Point c{0., 1., 0.};

    Triangle t{a, b, c};

    a[0] = 42.;  // mutate original point

    EXPECT_EQ(t.a, (Point{0., 0., 0.}));  // triangle vertex is independent of original point
}
