#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "geometry/utilities.hpp"

using namespace simple_slice::geometry;

// Test fixture for utilities
class UtilitiesTest : public ::testing::Test {
protected:
    static constexpr double kTestEpsilon = 1e-9;
};

//----------------------------------------------
// sign() function tests
//----------------------------------------------

TEST_F(UtilitiesTest, SignPositiveValue) {
    EXPECT_EQ(sign(1.), 1);
    EXPECT_EQ(sign(0.5), 1);
    EXPECT_EQ(sign(100.), 1);
    EXPECT_EQ(sign(1e-8), 1);
}

TEST_F(UtilitiesTest, SignNegativeValue) {
    EXPECT_EQ(sign(-1.), -1);
    EXPECT_EQ(sign(-0.5), -1);
    EXPECT_EQ(sign(-100.), -1);
    EXPECT_EQ(sign(-1e-8), -1);  // Just below negative default epsilon
}

TEST_F(UtilitiesTest, SignZeroValue) {
    EXPECT_EQ(sign(0.), 0);
}

TEST_F(UtilitiesTest, SignNearZeroWithinTolerance) {
    // Values within default kEpsilon (1e-9) should return 0
    EXPECT_EQ(sign(1e-10), 0);
    EXPECT_EQ(sign(-1e-10), 0);
    EXPECT_EQ(sign(5e-10), 0);
    EXPECT_EQ(sign(-5e-10), 0);
    EXPECT_EQ(sign(kEpsilon / 2.), 0);
    EXPECT_EQ(sign(-kEpsilon / 2.), 0);
}

TEST_F(UtilitiesTest, SignExactlyAtTolerance) {
    // Exactly at tolerance boundary should be treated as zero
    EXPECT_EQ(sign(kEpsilon), 0);
    EXPECT_EQ(sign(-kEpsilon), 0);
}

TEST_F(UtilitiesTest, SignJustAboveTolerance) {
    // Just above tolerance should be positive
    EXPECT_EQ(sign(kEpsilon + 1e-12), 1);
    EXPECT_EQ(sign(-kEpsilon - 1e-12), -1);
}

TEST_F(UtilitiesTest, SignCustomTolerance) {
    double custom_tol = 0.1;

    EXPECT_EQ(sign(0.5, custom_tol), 1.);     // Within custom tolerance
    EXPECT_EQ(sign(-0.5, custom_tol), -1.);   // Within custom tolerance
    EXPECT_EQ(sign(0.15, custom_tol), 1.);    // Above custom tolerance
    EXPECT_EQ(sign(-0.15, custom_tol), -1.);  // Below custom tolerance
}

TEST_F(UtilitiesTest, SignVeryLargeValues) {
    EXPECT_EQ(sign(1.e100), 1.);
    EXPECT_EQ(sign(-1.e100), -1.);
}

TEST_F(UtilitiesTest, SignInfinity) {
    EXPECT_EQ(sign(std::numeric_limits<double>::infinity()), 1.);
    EXPECT_EQ(sign(-std::numeric_limits<double>::infinity()), -1.);
}

TEST_F(UtilitiesTest, SignNaN) {
    // NaN comparisons are always false, so should return 0
    double nan_val = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(sign(nan_val), 0.);
}

TEST_F(UtilitiesTest, SignSymmetry) {
    // sign(-x) should equal -sign(x) for non-zero values
    double test_values[] = {0.5, 1., 10., 100., 1e-8};

    for (double val : test_values) {
        EXPECT_EQ(sign(-val), -sign(val));
    }
}

//----------------------------------------------
// clamp_to_unit_interval() function tests
//----------------------------------------------

TEST_F(UtilitiesTest, ClampValueWithinRange) {
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(0.), 0.);
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(0.5), 0.5);
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(1.), 1.);
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(0.25), 0.25);
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(0.75), 0.75);
}

TEST_F(UtilitiesTest, ClampValueBelowZero) {
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(-0.1), 0.);
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(-1.), 0.);
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(-100.), 0.);
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(-1e-10), 0.);
}

TEST_F(UtilitiesTest, ClampValueAboveOne) {
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(1.1), 1.);
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(2.), 1.);
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(100.), 1.);
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(1. + 1e-10), 1.);
}

TEST_F(UtilitiesTest, ClampBoundaries) {
    // Test exactly at boundaries
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(0.), 0.);
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(1.), 1.);
}

TEST_F(UtilitiesTest, ClampVerySmallPositive) {
    double very_small = 1e-100;
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(very_small), very_small);
}

TEST_F(UtilitiesTest, ClampVeryCloseToOne) {
    double almost_one = 1. - 1e-15;
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(almost_one), almost_one);
}

TEST_F(UtilitiesTest, ClampExtremeValues) {
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(-1e100), 0.);
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(1e100), 1.);
}

TEST_F(UtilitiesTest, ClampInfinity) {
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(std::numeric_limits<double>::infinity()), 1.);
    EXPECT_DOUBLE_EQ(clamp_to_unit_interval(-std::numeric_limits<double>::infinity()), 0.);
}

TEST_F(UtilitiesTest, ClampManyValues) {
    // Test a range of values
    for (double t = -0.5; t <= 1.5; t += 0.1) {
        double result = clamp_to_unit_interval(t);
        EXPECT_GE(result, 0.);
        EXPECT_LE(result, 1.);

        if (t < 0.) {
            EXPECT_DOUBLE_EQ(result, 0.);
        } else if (t > 1.) {
            EXPECT_DOUBLE_EQ(result, 1.);
        } else {
            EXPECT_NEAR(result, t, kTestEpsilon);
        }
    }
}

//----------------------------------------------
// Constants tests
//----------------------------------------------

TEST_F(UtilitiesTest, EpsilonValue) {
    EXPECT_DOUBLE_EQ(kEpsilon, 1e-9);
    EXPECT_GT(kEpsilon, 0.);
}

TEST_F(UtilitiesTest, EpsilonIsConstexpr) {
    // This test verifies kEpsilon can be used in constant expressions
    constexpr double test_epsilon = kEpsilon;
    EXPECT_DOUBLE_EQ(test_epsilon, 1e-9);
}
