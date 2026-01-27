#include <cmath>

#include <gtest/gtest.h>

#include "geometry/utilities.hpp"
#include "geometry/vector.hpp"

using namespace simple_slice::geometry;

// Test fixture for vector tests
class VectorTest : public ::testing::Test {
protected:
    // Helper: Check if two vectors are approximately equal
    void ExpectVectorNear(const Vector3D& actual, const Vector3D& expected) {
        EXPECT_NEAR(actual.x, expected.x, kEpsilon);
        EXPECT_NEAR(actual.y, expected.y, kEpsilon);
        EXPECT_NEAR(actual.z, expected.z, kEpsilon);
    }
};

//----------------------------------------------
// Constructor Tests
//----------------------------------------------

TEST_F(VectorTest, ConstructorWithDefaultZ) {
    Vector3D v{1., 2.};

    EXPECT_DOUBLE_EQ(v.x, 1.);
    EXPECT_DOUBLE_EQ(v.y, 2.);
    EXPECT_DOUBLE_EQ(v.z, 0.);
}

TEST_F(VectorTest, ConstructorExplicit3D) {
    Vector3D v{1., 2., 3.};

    EXPECT_DOUBLE_EQ(v.x, 1.);
    EXPECT_DOUBLE_EQ(v.y, 2.);
    EXPECT_DOUBLE_EQ(v.z, 3.);
}

//----------------------------------------------
// Addition Tests
//----------------------------------------------

TEST_F(VectorTest, AdditionBasic) {
    Vector3D v1{1., 2., 3.};
    Vector3D v2{4., 5., 6.};

    Vector3D result = v1 + v2;

    ExpectVectorNear(result, Vector3D{5., 7., 9.});
}

TEST_F(VectorTest, AdditionWithZeroVector) {
    Vector3D v{1., 2., 3.};
    Vector3D zero{0., 0., 0.};

    Vector3D result = v + zero;

    ExpectVectorNear(result, v);
}

TEST_F(VectorTest, AdditionIsCommutative) {
    Vector3D v1{1., 2., 3.};
    Vector3D v2{4., 5., 6.};

    ExpectVectorNear(v1 + v2, v2 + v1);
}

TEST_F(VectorTest, AdditionIsAssociative) {
    Vector3D v1{1., 2., 3.};
    Vector3D v2{4., 5., 6.};
    Vector3D v3{7., 8., 9.};

    ExpectVectorNear((v1 + v2) + v3, v1 + (v2 + v3));
}

//----------------------------------------------
// Subtraction Tests
//----------------------------------------------

TEST_F(VectorTest, SubtractionBasic) {
    Vector3D v1{4., 5., 6.};
    Vector3D v2{1., 2., 3.};

    Vector3D result = v1 - v2;

    ExpectVectorNear(result, Vector3D{3., 3., 3.});
}

TEST_F(VectorTest, SubtractionWithSelf) {
    Vector3D v{1., 2., 3.};

    Vector3D result = v - v;

    ExpectVectorNear(result, Vector3D{0., 0., 0.});
}

TEST_F(VectorTest, SubtractionNegation) {
    Vector3D v{1., 2., 3.};
    Vector3D zero{0., 0., 0.};

    Vector3D result = zero - v;

    ExpectVectorNear(result, Vector3D{-1., -2., -3.});
}

//----------------------------------------------
// Scalar Multiplication Tests
//----------------------------------------------

TEST_F(VectorTest, ScalarMultiplicationVectorTimesScalar) {
    Vector3D v{1., 2., 3.};

    Vector3D result = v * 2.5;

    ExpectVectorNear(result, Vector3D{2.5, 5., 7.5});
}

TEST_F(VectorTest, ScalarMultiplicationScalarTimesVector) {
    Vector3D v{1., 2., 3.};

    Vector3D result = 2.5 * v;

    ExpectVectorNear(result, Vector3D{2.5, 5.0, 7.5});
}

TEST_F(VectorTest, ScalarMultiplicationCommutative) {
    Vector3D v{1., 2., 3.};
    double s = 2.5;

    ExpectVectorNear(v * s, s * v);
}

TEST_F(VectorTest, ScalarMultiplicationByZero) {
    Vector3D v{1., 2., 3.};

    Vector3D result = v * 0.0;

    ExpectVectorNear(result, Vector3D{0., 0., 0.});
}

TEST_F(VectorTest, ScalarMultiplicationByNegative) {
    Vector3D v{1., 2., 3.};

    Vector3D result = v * -1.;

    ExpectVectorNear(result, Vector3D{-1., -2., -3.});
}

TEST_F(VectorTest, ScalarMultiplicationByOne) {
    Vector3D v{1., 2., 3.};

    Vector3D result = v * 1.;

    ExpectVectorNear(result, v);
}

TEST_F(VectorTest, ScalarMultiplicationDistributive) {
    Vector3D v1{1., 2., 3.};
    Vector3D v2{4., 5., 6.};
    double s = 2.5;

    // s * (v1 + v2) = s*v1 + s*v2
    ExpectVectorNear(s * (v1 + v2), (s * v1) + (s * v2));
}

//----------------------------------------------
// Dot Product Tests
//----------------------------------------------

TEST_F(VectorTest, DotProductOrthogonalVectors) {
    Vector3D v1{1., 0., 0.};
    Vector3D v2{0., 1., 0.};

    double result = dot_product(v1, v2);

    EXPECT_DOUBLE_EQ(result, 0.);
}

TEST_F(VectorTest, DotProductParallelVectors) {
    Vector3D v1{1., 2., 3.};
    Vector3D v2{2., 4., 6.};

    double result = dot_product(v1, v2);

    EXPECT_NEAR(result, 28., kEpsilon);  // 2 + 8 + 18
}

TEST_F(VectorTest, DotProductOppositeVectors) {
    Vector3D v1{1., 0., 0.};
    Vector3D v2{-1., 0., 0.};

    double result = dot_product(v1, v2);

    EXPECT_DOUBLE_EQ(result, -1.);
}

TEST_F(VectorTest, DotProductIsCommutative) {
    Vector3D v1{1., 2., 3.};
    Vector3D v2{4., 5., 6.};

    EXPECT_DOUBLE_EQ(dot_product(v1, v2), dot_product(v2, v1));
}

TEST_F(VectorTest, DotProductWithSelf) {
    Vector3D v{3., 4., 0.};

    double result = dot_product(v, v);

    EXPECT_NEAR(result, 25., kEpsilon);  // 9 + 16
}

TEST_F(VectorTest, DotProduct2DVectors) {
    Vector3D v1{3., 4.};
    Vector3D v2{1., 2.};

    double result = dot_product(v1, v2);

    EXPECT_NEAR(result, 11., kEpsilon);  // 3 + 8
}

//----------------------------------------------
// Cross Product Tests
//----------------------------------------------

TEST_F(VectorTest, CrossProductUnitVectorsIxJ) {
    Vector3D i{1., 0., 0.};
    Vector3D j{0., 1., 0.};

    Vector3D k = cross_product(i, j);

    ExpectVectorNear(k, Vector3D{0., 0., 1.});
}

TEST_F(VectorTest, CrossProductUnitVectorsJxI) {
    Vector3D i{1., 0., 0.};
    Vector3D j{0., 1., 0.};

    Vector3D neg_k = cross_product(j, i);

    ExpectVectorNear(neg_k, Vector3D{0., 0., -1.});
}

TEST_F(VectorTest, CrossProductIsAnticommutative) {
    Vector3D v1{1., 2., 3.};
    Vector3D v2{4., 5., 6.};

    Vector3D result1 = cross_product(v1, v2);
    Vector3D result2 = cross_product(v2, v1);

    ExpectVectorNear(result1, result2 * -1.);
}

TEST_F(VectorTest, CrossProductParallelVectorsIsZero) {
    Vector3D v1{1., 2., 3.};
    Vector3D v2{2., 4., 6.};  // v2 = 2 * v1

    Vector3D result = cross_product(v1, v2);

    ExpectVectorNear(result, Vector3D{0., 0., 0.});
}

TEST_F(VectorTest, CrossProductIsPerpendicular) {
    Vector3D v1{1., 2., 3.};
    Vector3D v2{4., 5., 6.};

    Vector3D result = cross_product(v1, v2);

    // Cross product should be perpendicular to both input vectors
    EXPECT_NEAR(dot_product(result, v1), 0., kEpsilon);
    EXPECT_NEAR(dot_product(result, v2), 0., kEpsilon);
}

TEST_F(VectorTest, CrossProduct2DGivesSignedArea) {
    Vector3D v1{3., 0.};  // Along x-axis
    Vector3D v2{0., 4.};  // Along y-axis

    Vector3D result = cross_product(v1, v2);

    // Z-component gives signed area of parallelogram: 3 * 4 = 12
    EXPECT_DOUBLE_EQ(result.x, 0.);
    EXPECT_DOUBLE_EQ(result.y, 0.);
    EXPECT_NEAR(result.z, 12., kEpsilon);
}

TEST_F(VectorTest, CrossProductWithSelfIsZero) {
    Vector3D v{1., 2., 3.};

    Vector3D result = cross_product(v, v);

    ExpectVectorNear(result, Vector3D{0., 0., 0.});
}

//----------------------------------------------
// Magnitude Tests
//----------------------------------------------

TEST_F(VectorTest, MagnitudeUnitVectorX) {
    Vector3D i{1., 0., 0.};

    EXPECT_NEAR(magnitude(i), 1., kEpsilon);
}

TEST_F(VectorTest, MagnitudeUnitVectorY) {
    Vector3D j{0., 1., 0.};

    EXPECT_NEAR(magnitude(j), 1., kEpsilon);
}

TEST_F(VectorTest, MagnitudeUnitVectorZ) {
    Vector3D k{0., 0., 1.};

    EXPECT_NEAR(magnitude(k), 1., kEpsilon);
}

TEST_F(VectorTest, Magnitude345RightTriangle) {
    Vector3D v{3., 4.};

    EXPECT_NEAR(magnitude(v), 5., kEpsilon);
}

TEST_F(VectorTest, MagnitudeZeroVector) {
    Vector3D zero{0., 0., 0.};

    EXPECT_DOUBLE_EQ(magnitude(zero), 0.);
}

TEST_F(VectorTest, MagnitudeNegativeComponents) {
    Vector3D v1{3., 4., 0.};
    Vector3D v2{-3., -4., 0.};

    EXPECT_NEAR(magnitude(v1), magnitude(v2), kEpsilon);
}

TEST_F(VectorTest, Magnitude3DPythagorean) {
    Vector3D v{1., 2., 2.};  // sqrt(1 + 4 + 4) = 3

    EXPECT_NEAR(magnitude(v), 3., kEpsilon);
}

TEST_F(VectorTest, MagnitudeArbitraryVector) {
    Vector3D v{2., 3., 6.};  // sqrt(4 + 9 + 36) = 7

    EXPECT_NEAR(magnitude(v), 7., kEpsilon);
}

//----------------------------------------------
// Distance Tests
//----------------------------------------------

TEST_F(VectorTest, DistanceBetweenSamePoint) {
    Vector3D v{1., 2., 3.};

    EXPECT_DOUBLE_EQ(distance(v, v), 0.);
}

TEST_F(VectorTest, DistanceIsSymmetric) {
    Vector3D v1{1., 2., 3.};
    Vector3D v2{4., 5., 6.};

    EXPECT_NEAR(distance(v1, v2), distance(v2, v1), kEpsilon);
}

TEST_F(VectorTest, DistanceUnitAlongXAxis) {
    Vector3D origin{0., 0., 0.};
    Vector3D x_unit{1., 0., 0.};

    EXPECT_NEAR(distance(origin, x_unit), 1., kEpsilon);
}

TEST_F(VectorTest, Distance345Triangle) {
    Vector3D v1{0., 0.};
    Vector3D v2{3., 4.};

    EXPECT_NEAR(distance(v1, v2), 5., kEpsilon);
}

TEST_F(VectorTest, Distance3D) {
    Vector3D v1{1., 2., 3.};
    Vector3D v2{4., 6., 8.};  // Difference: (3, 4, 5)

    // sqrt(9 + 16 + 25) = sqrt(50)
    EXPECT_NEAR(distance(v1, v2), std::sqrt(50.), kEpsilon);
}

TEST_F(VectorTest, DistanceArbitrary) {
    Vector3D v1{-1., -2., -3.};
    Vector3D v2{2., 2., 1.};  // Difference: (3, 4, 4)

    // sqrt(9 + 16 + 16) = sqrt(41)
    EXPECT_NEAR(distance(v1, v2), std::sqrt(41.), kEpsilon);
}
