#pragma once

#include "geometry.hpp"
#include "triangle_mesh.hpp"

namespace simple_slice::mesh {

/**
 * @brief Flips the orientation of a triangle
 *
 * The triangle is flipped by swapping its second and third vertices, reversing its orientation while
 * preserving its geometry.
 *
 * @param triangle Triangle to be flipped in place
 */
void flip_triangle(Triangle&);

/**
 * @brief Checks whether a triangle contains a directed edge
 *
 * The function returns true if the triangle traverses the edge from the first point to the second
 * point in its vertex ordering.
 *
 * @param triangle Triangle to inspect
 * @param from Starting vertex of the directed edge
 * @param to Ending vertex of the directed edge
 * @return true if the directed edge is present; false otherwise
 */
bool has_directed_edge(const Triangle&, const Point&, const Point&);

/**
 * @brief Checks whether two triangles have consistent orientations across a shared edge
 *
 * Two triangles are considered consistently oriented if they traverse their shared edge in opposite
 * directions.
 *
 * @param t1 First triangle
 * @param t2 Second triangle
 * @param edge Shared edge in canonical form
 * @return true if the orientations are consistent; false otherwise
 */
bool are_orientations_consistent(const Triangle&, const Triangle&, const Edge&);

/**
 * @brief Reorients triangles in a mesh that have inconsistent orientations
 *
 * Starting from a seed triangle with known correct orientation, this function traverses the
 * connected component of the mesh and flips triangles as needed to enforce consistent orientation
 * across shared edges.
 *
 * Only triangles in the connected component containing the seed triangle are processed.
 *
 * @param mesh Mesh whose triangles are to be reoriented
 * @param seed Index of the seed triangle
 * @return List of triangles that were reoriented
 */
std::vector<Triangle> reorient_inconsistent_triangles(TriangleMesh&, std::size_t seed);

/**
 * @brief Exports triangles with inconsistent orientations to an output stream
 *
 * The function identifies triangles with inconsistent orientation relative to the provided seed
 * triangle and writes them to the output stream in ASCII STL format.
 *
 * @param mesh Mesh containing the triangles
 * @param seed Index of the seed triangle
 * @param out Output stream to write the exported triangles to
 */
void export_inconsistent_triangles(TriangleMesh&, std::size_t seed, std::ostream& out);

}  // namespace simple_slice::mesh
