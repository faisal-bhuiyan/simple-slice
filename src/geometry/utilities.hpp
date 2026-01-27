#pragma once

namespace simple_slice::geometry {

/**
 * @brief Numerical tolerance for floating-point comparisons
 *
 * Used throughout the library to handle floating-point rounding errors.
 * A value is considered "zero" if its absolute value is < kEpsilon.
 */
constexpr double kEpsilon{1e-9};

/**
 * @brief Robust sign test for floating-point values
 *
 * Classifies a value as positive (+1), negative (-1), or zero (0) with
 * tolerance to avoid spurious results from floating-point error.
 *
 * @param value Value to test
 * @param tolerance Tolerance threshold (default: kEpsilon)
 * @return +1 if value > tolerance, -1 if value < -tolerance, 0 if |value| ≤ tolerance
 */
inline int sign(double value, double tolerance = kEpsilon) {
    if (value > tolerance) {
        return 1;
    }
    if (value < -tolerance) {
        return -1;
    }
    return 0;
}

/**
 * @brief Clamp a value to the closed interval [0, 1]
 *
 * @param value Value to clamp
 * @return 0 if t < 0, 1 if t > 1, otherwise t
 *
 * @note Useful for clamping line parameters to segment endpoints.
 */
inline double clamp_to_unit_interval(double value) {
    return value < 0. ? 0. : (value > 1. ? 1. : value);
}

}  // namespace simple_slice::geometry
