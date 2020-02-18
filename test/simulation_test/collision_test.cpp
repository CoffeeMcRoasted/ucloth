#include <gtest/gtest.h>
#include <simulation/collision.h>
#include <simulation/constraint.h>
#include <simulation/pbdsystem.h>
#include <simulation/world.h>

class Collision_test : public ::testing::Test
{
protected:
    void SetUp() override
    {
    }
    void TearDown() override
    {
        world.clear();
    }
    ucloth::simulation::World world;
    ucloth::umath::Real delta_time = 0.033;
    ucloth::umath::Real cloth_thickness = 0.01;
};

TEST_F(Collision_test, Collision_Generation_test)
{
    using namespace ucloth;

    world.positions = {
        {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {1.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {1.0f / 3.0f, 1.0f / 3.0f, 0.05}};
    world.velocities = {{0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, 1.0}, {0.0, 0.0, -50.0}};
    world.inverse_particle_masses = {1 / 0.1, 1 / 0.1, 1 / 0.1, 1 / 0.1, 1 / 0.1};
    world.meshes = {{{0, 3, 1}, {1, 3, 2}}};  //< 1 quad composed of 2 triangles.

    std::vector<umath::Position> pos_estimates;
    simulation::PBD_system::calculate_position_estimates(world.positions, world.velocities, delta_time, pos_estimates);

    std::vector<simulation::Collision_constraint> collisions = simulation::PBD_system::generate_collision_constraints(
        world.positions, pos_estimates, world.meshes, cloth_thickness);

    ASSERT_EQ(collisions.size(), 1);
    simulation::Collision_constraint const& c = collisions.front();
    EXPECT_EQ(c.q, 4);
    EXPECT_EQ(c.p1, 0);
    EXPECT_EQ(c.p2, 3);
    EXPECT_EQ(c.p3, 1);

    simulation::project_collision_constraints(collisions, world.inverse_particle_masses, 100, pos_estimates);
    // Check that the collision point is ABOVE the triangle after iteration
    EXPECT_TRUE(pos_estimates[c.q].z > pos_estimates[c.p1].z);
}

TEST_F(Collision_test, Inverse_Collision_Generation_test)
{
    using namespace ucloth;
    // Compared to the previous test:
    //  - We invert the position of the colliding particle in the z axis
    //  - We invert all the velocities in the z axis
    world.positions = {
        {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {1.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {1.0f / 3.0f, 1.0f / 3.0f, -0.05}};
    world.velocities = {{0.0, 0.0, -1.0}, {0.0, 0.0, -1.0}, {0.0, 0.0, -1.0}, {0.0, 0.0, -1.0}, {0.0, 0.0, 50.0}};
    world.inverse_particle_masses = {1 / 0.1, 1 / 0.1, 1 / 0.1, 1 / 0.1, 1 / 0.1};
    world.meshes = {{{0, 3, 1}, {1, 3, 2}}};  //< 1 quad composed of 2 triangles.

    std::vector<umath::Position> pos_estimates;
    simulation::PBD_system::calculate_position_estimates(world.positions, world.velocities, delta_time, pos_estimates);

    std::vector<simulation::Collision_constraint> collisions = simulation::PBD_system::generate_collision_constraints(
        world.positions, pos_estimates, world.meshes, cloth_thickness);

    ASSERT_EQ(collisions.size(), 1);
    simulation::Collision_constraint const& c = collisions.front();
    EXPECT_EQ(c.q, 4);
    EXPECT_EQ(c.p1, 0);
    EXPECT_EQ(c.p2, 3);
    EXPECT_EQ(c.p3, 1);

    simulation::project_collision_constraints(collisions, world.inverse_particle_masses, 100, pos_estimates);
    // Check that the collision point is BELOW the triangle after iteration
    EXPECT_TRUE(pos_estimates[c.q].z < pos_estimates[c.p1].z);
}