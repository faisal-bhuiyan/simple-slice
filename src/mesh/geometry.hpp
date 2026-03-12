#pragma once

#include <array>
#include <cstddef>
#include <functional>
#include <utility>

namespace simple_slice::mesh {

//----------------------------------------------
// Point
//----------------------------------------------

/// A 3D point represented by Cartesian coordinates (x, y, z)
using Point = std::array<double, 3>;

/**
 * @brief Hash functor for Point
 *
 * Computes a combined hash of the three coordinate values using a boost-style hash combination
 * technique.
 *
 * @note `noexcept` allows the compiler and library to optimize more aggressively.
 */
struct PointHash {
    std::size_t operator()(const Point& p) const noexcept {
        std::size_t h1 = std::hash<double>{}(p[0]);  // hash x coordinate
        std::size_t h2 = std::hash<double>{}(p[1]);  // hash y coordinate
        std::size_t h3 = std::hash<double>{}(p[2]);  // hash z coordinate

        // Combine hashes using boost-style technique
        std::size_t hash = h1;
        hash ^= h2 + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
        hash ^= h3 + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
        return hash;
    }
};

/**
 * @brief Equality functor for Point
 *
 * Points are considered equal if all three coordinates compare equal.
 *
 * @note `noexcept` allows the compiler and library to optimize more aggressively.
 */
struct PointEquality {
    bool operator()(const Point& a, const Point& b) const noexcept {
        return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
    }
};

//----------------------------------------------
// Edge
//----------------------------------------------

/**
 * @brief An edge represented in canonical form
 *
 * The two endpoints are ordered such that the first point is lexicographically less than or equal to
 * the second. Lexicographic ordering means that the x-coordinate is the primary key, followed by the
 * y-coordinate, and then the z-coordinate.
 */
using Edge = std::pair<Point, Point>;

/**
 * @brief Builds a canonical edge from two points
 *
 * The returned edge is ordered such that the smaller point (lexicographically) appears first.
 *
 * @param p First endpoint
 * @param q Second endpoint
 * @return Canonicalized edge
 */
inline Edge make_edge(const Point& p, const Point& q) {
    return (p < q) ? Edge{p, q} : Edge{q, p};
}

/**
 * @brief Hash functor for Edge
 *
 * Combines the hashes of the two endpoint points using a boost-style hash combination technique.
 *
 * @note `noexcept` allows the compiler and library to optimize more aggressively.
 */
struct EdgeHash {
    std::size_t operator()(const Edge& e) const noexcept {
        PointHash point_hash;
        std::size_t h1 = point_hash(e.first);
        std::size_t h2 = point_hash(e.second);

        // Combine hashes using boost-style technique
        std::size_t hash = h1;
        hash ^= h2 + 0x9e3779b97f4a7c15ULL + (hash << 6) + (hash >> 2);
        return hash;
    }
};

/**
 * @brief Equality functor for Edge
 *
 * Two edges are equal if both corresponding endpoints compare equal. Edges are assumed to be in
 * canonical form.
 *
 * @note `noexcept` allows the compiler and library to optimize more aggressively.
 */
struct EdgeEquality {
    bool operator()(const Edge& e1, const Edge& e2) const noexcept {
        PointEquality eq;
        return eq(e1.first, e2.first) && eq(e1.second, e2.second);
    }
};

//----------------------------------------------
// Triangle
//----------------------------------------------

/**
 * @brief Triangle in 3D for storing STL mesh data
 */
struct Triangle {
    Point a;  //< first vertex
    Point b;  //< second vertex
    Point c;  //< third vertex
};

}  // namespace simple_slice::mesh
