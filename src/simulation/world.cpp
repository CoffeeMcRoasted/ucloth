#include "world.h"

#include <algorithm>
#include <unordered_set>
#include <unordered_map>

namespace ucloth
{
namespace simulation
{
struct pair_hash
{
    template <typename T1, typename T2>
    std::size_t operator()(std::pair<T1, T2> const& pair) const
    {
        return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
    }
};

void World::clear()
{
    positions.clear();
    velocities.clear();
    inverse_particle_masses.clear();
    distance_constraints.clear();
    bending_constraints.clear();
    accelerations.clear();
    meshes.clear();
}
void World::reserve_for_particles(size_t const n_particles)
{
    positions.reserve(n_particles);
    velocities.reserve(n_particles);
    inverse_particle_masses.reserve(n_particles);
}

void World::add_acceleration(umath::Vec3 acceleration)
{
    accelerations.emplace_back(acceleration);
}

void World::add_cloth(std::vector<umath::Position> const& pos,
                      Mesh const& mesh,
                      umath::Real cloth_elasticity,
                      umath::Real cloth_bending_stifness)
{
    size_t const new_particles = pos.size();
    size_t const current_size = positions.size();
    reserve_for_particles(current_size + new_particles);
    positions.insert(positions.end(), pos.begin(), pos.end());
    velocities.resize(current_size + new_particles, umath::Vec3{0.0f, 0.0f, 0.0f});
    // Current size should correspond to the latest available index
    size_t const index_shift = current_size;
    Mesh copy = mesh;
    for (auto& face : copy)
    {
        face[0] += index_shift;
        face[1] += index_shift;
        face[2] += index_shift;
    }
    auto const& emplaced_mesh = meshes.emplace_back(std::move(copy));
    // Generate distance constraints
    // Generate edges
    std::unordered_map<Edge, std::vector<Face>, pair_hash> edge_faces_map;
    // Worst case scenario all triangles are separate
    edge_faces_map.reserve(new_particles * 3);
    for (auto const& face : emplaced_mesh)
    {
        Edge const e0 = {face[0], face[1]};
        Edge const e1 = {face[1], face[2]};
        Edge const e2 = {face[2], face[0]};
        Edge const ei0 = {face[1], face[0]};
        Edge const ei1 = {face[2], face[1]};
        Edge const ei2 = {face[0], face[2]};
        // Find edge to add face to.
        // We cannot ensure the order of the edge vertices.
        // E0
        if (edge_faces_map.find(e0) == edge_faces_map.end())
        {
            edge_faces_map[e0].push_back(face);
        }
        else if (edge_faces_map.find(ei0) == edge_faces_map.end())
        {
            edge_faces_map[ei0].push_back(face);
        }
        else
        {
            edge_faces_map.emplace(std::make_pair(e0, std::vector<Face>{face}));
        }
        // E1
        if (edge_faces_map.find(e1) == edge_faces_map.end())
        {
            edge_faces_map[e1].push_back(face);
        }
        else if (edge_faces_map.find(ei1) == edge_faces_map.end())
        {
            edge_faces_map[ei1].push_back(face);
        }
        else
        {
            edge_faces_map.emplace(std::make_pair(e1, std::vector<Face>{face}));
        }
        // E2
        if (edge_faces_map.find(e2) == edge_faces_map.end())
        {
            edge_faces_map[e2].push_back(face);
        }
        else if (edge_faces_map.find(ei2) == edge_faces_map.end())
        {
            edge_faces_map[ei2].push_back(face);
        }
        else
        {
            edge_faces_map.emplace(std::make_pair(e2, std::vector<Face>{face}));
        }
    }
    for (auto const& pair : edge_faces_map)
    {
        auto const& edge = pair.first;
        umath::Real const distance = umath::length(positions[edge.first] - positions[edge.second]);
        distance_constraints.emplace_back(Distance_constraint{edge.first, edge.second, distance, cloth_elasticity});

        auto const& faces = pair.second;
        // We should never have a non-manifold mesh, with more than 2 faces per edge
        if (faces.size() > 1)
        {
            Particle const& p1 = edge.first;
            Particle const& p2 = edge.second;
            Particle const& p3 = *std::find_if(
                faces[0].begin(), faces[0].end(), [&p1, &p2](Particle const& p) { return p != p1 && p != p2; });
            Particle const& p4 = *std::find_if(
                faces[1].begin(), faces[1].end(), [&p1, &p2](Particle const& p) { return p != p1 && p != p2; });

            umath::Vec3 const n1 = umath::cross(positions[p3] - positions[p1], positions[p2] - positions[p1]);
            umath::Vec3 const n2 = umath::cross(positions[p4] - positions[p1], positions[p2] - positions[p1]);

            umath::Real const dihedral_angle = acosf(umath::dot(n1, n2) / (umath::length(n1) * umath::length(n2)));
            bending_constraints.emplace_back(
                Bending_constraint{p1, p2, p3, p4, dihedral_angle, cloth_bending_stifness});
        }
    }
}

}  // namespace simulation
}  // namespace ucloth