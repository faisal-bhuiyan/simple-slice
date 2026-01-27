#pragma once

#include <cmath>

namespace simple_slice::geometry {

/**
 * @brief A 3D vector in Cartesian coordinates
 *
 * Stores x, y, z components and provides basic vector arithmetic operations.
 * This struct is used for both 3D geometry and 2D operations (setting z=0).
 */
struct Vector3D {
    double x, y, z;  ///< Cartesian coordinates (x, y, z)

    /**
     * @brief Construct a vector with given coordinates
     * @param x X-component
     * @param y Y-component
     * @param z Z-component
     */
    Vector3D(double x, double y, double z = 0.) : x(x), y(y), z(z) {}

    /**
     * @brief Vector addition
     * @param other Vector to add
     * @return Component-wise sum of this vector and other
     */
    Vector3D operator+(const Vector3D& other) const {
        return {this->x + other.x, this->y + other.y, this->z + other.z};
    }

    /**
     * @brief Vector subtraction
     * @param other Vector to subtract
     * @return Component-wise difference of this vector and other
     */
    Vector3D operator-(const Vector3D& other) const {
        return {this->x - other.x, this->y - other.y, this->z - other.z};
    }

    /**
     * @brief Scalar multiplication
     * @param s Scalar multiplier
     * @return Vector with each component multiplied by s
     */
    Vector3D operator*(double s) const { return {this->x * s, this->y * s, this->z * s}; }
};

/**
 * @brief Scalar multiplication (scalar * vector).
 * @param s Scalar multiplier
 * @param v Vector to multiply
 * @return Vector with each component multiplied by s
 * @note Enables `2.0 * v` syntax in addition to `v * 2.0`
 */
inline Vector3D operator*(double s, const Vector3D& v) {
    return v * s;
}

/**
 * @brief Compute the dot product (scalar product) of two vectors
 *
 * @param v_1 First vector
 * @param v_2 Second vector
 * @return Dot product: v_1 · v_2 = v_1.x*v_2.x + v_1.y*v_2.y + v_1.z*v_2.z
 *
 * @note The dot product measures the projection of one vector onto another.
 *       When vectors are normalized, result ∈ [-1,1] represents cos(angle).
 */
inline double dot_product(const Vector3D& v_1, const Vector3D& v_2) {
    return v_1.x * v_2.x + v_1.y * v_2.y + v_1.z * v_2.z;
}

/**
 * @brief Compute the cross product (vector product) of two vectors
 *
 * @param v1 First vector
 * @param v2 Second vector
 * @return Cross product: v1 × v2 (perpendicular to both vectors)
 *
 * @note The magnitude of the result equals the area of the parallelogram
 *       formed by a and b. Direction follows the right-hand rule.
 *       For 2D vectors (z=0), the z-component gives the signed area.
 */
inline Vector3D cross_product(const Vector3D& v1, const Vector3D& v2) {
    return {
        v1.y * v2.z - v1.z * v2.y,  // x component
        v1.z * v2.x - v1.x * v2.z,  // y component
        v1.x * v2.y - v1.y * v2.x   // z component
    };
}

/**
 * @brief Compute the Euclidean magnitude (length) of a vector
 *
 * @param v Vector to measure
 * @return ||v|| = sqrt(v.x² + v.y² + v.z²)
 */
inline double magnitude(const Vector3D& v) {
    return std::sqrt(dot_product(v, v));
}

/**
 * @brief Compute the Euclidean distance between two position vectors
 *
 * @param v1 First vector
 * @param v2 Second vector
 * @return ||v2 - v1|| (distance from v1 to v2)
 */
inline double distance(const Vector3D& v1, const Vector3D& v2) {
    return magnitude(v1 - v2);
}

}  // namespace simple_slice::geometry
