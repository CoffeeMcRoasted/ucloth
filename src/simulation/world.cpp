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

void World::add_constraints_for_mesh(Mesh const& mesh, umath::Real cloth_elasticity, umath::Real cloth_bending_stifness)
{
    // Generate distance constraints
    // Generate edges
    // std::unordered_map<Edge, std::vector<Face>, pair_hash> edge_faces_map;
    std::vector<std::pair<Edge, std::vector<Face>>> edge_faces;
    // Worst case scenario all triangles are separate
    edge_faces.reserve((mesh.end - mesh.begin) * 3);
    for (auto const& face : mesh.faces)
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
        auto edge0_face_it = std::find_if(
            edge_faces.begin(), edge_faces.end(), [&e0, &ei0](std::pair<Edge, std::vector<Face>> const& edge_face) {
                return edge_face.first == e0 || edge_face.first == ei0;
            });

        if (edge0_face_it != edge_faces.end())
        {
            edge0_face_it->second.push_back(face);
        }
        else
        {
            edge_faces.emplace_back(std::make_pair(e0, std::vector<Face>{face}));
        }
        // E1
        auto edge1_face_it = std::find_if(
            edge_faces.begin(), edge_faces.end(), [&e1, &ei1](std::pair<Edge, std::vector<Face>> const& edge_face) {
                return edge_face.first == e1 || edge_face.first == ei1;
            });

        if (edge1_face_it != edge_faces.end())
        {
            edge1_face_it->second.push_back(face);
        }
        else
        {
            edge_faces.emplace_back(std::make_pair(e1, std::vector<Face>{face}));
        }
        // E2
        auto edge2_face_it = std::find_if(
            edge_faces.begin(), edge_faces.end(), [&e2, &ei2](std::pair<Edge, std::vector<Face>> const& edge_face) {
                return edge_face.first == e2 || edge_face.first == ei2;
            });

        if (edge2_face_it != edge_faces.end())
        {
            edge2_face_it->second.push_back(face);
        }
        else
        {
            edge_faces.emplace_back(std::make_pair(e2, std::vector<Face>{face}));
        }
    }
    for (auto const& pair : edge_faces)
    {
        auto const& edge = pair.first;
        umath::Real const distance = umath::length(positions[edge.first] - positions[edge.second]);
        distance_constraints.emplace_back(Distance_constraint{edge.first, edge.second, distance, cloth_elasticity});

        auto const& faces = pair.second;
        // We should never have a non-manifold mesh, with more than 2 faces per edge
        // We dont generate bending constraints for edges associated to a single face
        if (faces.size() == 2)
        {
            Particle const& p1 = edge.first;
            Particle const& p2 = edge.second;
            Particle const& p3 = *std::find_if(
                faces[0].begin(), faces[0].end(), [&p1, &p2](Particle const& p) { return p != p1 && p != p2; });
            Particle const& p4 = *std::find_if(
                faces[1].begin(), faces[1].end(), [&p1, &p2](Particle const& p) { return p != p1 && p != p2; });

            umath::Vec3 const p2p1 = positions[p2] - positions[p1];
            umath::Vec3 const p3p1 = positions[p3] - positions[p1];
            umath::Vec3 const p4p1 = positions[p4] - positions[p1];

            umath::Vec3 const n1 = umath::normalize(umath::cross(p2p1, p3p1));
            umath::Vec3 const n2 = umath::normalize(umath::cross(p2p1, p4p1));
            // with normalized vector the dot product equals the cosine of the angle.
            umath::Real const dihedral_angle = acosf(umath::dot(n1, n2));
            bending_constraints.emplace_back(
                Bending_constraint{p1, p2, p3, p4, dihedral_angle, cloth_bending_stifness});
        }
    }
}

Mesh const& World::add_cloth(std::vector<umath::Position> const& pos,
                             Mesh const& mesh,
                             umath::Real cloth_mass,
                             umath::Real cloth_elasticity,
                             umath::Real cloth_bending_stifness)
{
    size_t const new_particles = pos.size();
    size_t const current_size = positions.size();
    reserve_for_particles(current_size + new_particles);
    positions.insert(positions.end(), pos.begin(), pos.end());
    velocities.resize(current_size + new_particles, umath::Vec3{0.0f, 0.0f, 0.0f});
    umath::Real const inv_mass_per_particle = static_cast<umath::Real>(current_size + new_particles) / cloth_mass;
    inverse_particle_masses.resize(current_size + new_particles, inv_mass_per_particle);
    // Current size should correspond to the latest available index
    size_t const index_shift = current_size;
    Mesh copy = mesh;
    copy.begin += index_shift;
    copy.end += index_shift;
    for (auto& face : copy.faces)
    {
        face[0] += index_shift;
        face[1] += index_shift;
        face[2] += index_shift;
    }
    copy.type = Mesh_type::cloth;
    auto const& emplaced_mesh = meshes.emplace_back(std::move(copy));
    add_constraints_for_mesh(emplaced_mesh, cloth_elasticity, cloth_bending_stifness);
    return emplaced_mesh;
}

Mesh const& World::add_static_mesh(std::vector<umath::Position> const& pos, Mesh const& mesh)
{
    size_t const new_particles = pos.size();
    size_t const current_size = positions.size();
    reserve_for_particles(current_size + new_particles);
    positions.insert(positions.end(), pos.begin(), pos.end());
    velocities.resize(current_size + new_particles, umath::Vec3{0.0f, 0.0f, 0.0f});
    inverse_particle_masses.resize(current_size + new_particles, 0.0f);
    // Current size should correspond to the latest available index
    size_t const index_shift = current_size;
    Mesh copy = mesh;
    copy.begin += index_shift;
    copy.end += index_shift;
    for (auto& face : copy.faces)
    {
        face[0] += index_shift;
        face[1] += index_shift;
        face[2] += index_shift;
    }
    copy.type = Mesh_type::static_mesh;
    auto const& emplaced_mesh = meshes.emplace_back(std::move(copy));
    add_constraints_for_mesh(emplaced_mesh, 1.0f, 1.0f);
    return emplaced_mesh;
}

Mesh const& World::add_rigid_body(std::vector<umath::Position> const& pos, Mesh const& mesh, umath::Real mass)
{
    size_t const new_particles = pos.size();
    size_t const current_size = positions.size();
    reserve_for_particles(current_size + new_particles);
    positions.insert(positions.end(), pos.begin(), pos.end());
    velocities.resize(current_size + new_particles, umath::Vec3{0.0f, 0.0f, 0.0f});
    inverse_particle_masses.resize(current_size + new_particles, 1.0 / mass);
    // Current size should correspond to the latest available index
    size_t const index_shift = current_size;
    Mesh copy = mesh;
    copy.begin += index_shift;
    copy.end += index_shift;
    for (auto& face : copy.faces)
    {
        face[0] += index_shift;
        face[1] += index_shift;
        face[2] += index_shift;
    }
    copy.type = Mesh_type::rigid_body;
    auto const& emplaced_mesh = meshes.emplace_back(std::move(copy));
    add_constraints_for_mesh(emplaced_mesh, 1.0f, 1.0f);
    return emplaced_mesh;
}

void World::attach_particle(Mesh const& mesh, Particle particle, umath::Position pos)
{
    Particle particle_in_world = mesh.begin + particle;
    particle_attachments.push_back(
        std::move(Attachment{particle_in_world, inverse_particle_masses[particle_in_world], pos}));
}

}  // namespace simulation
}  // namespace ucloth