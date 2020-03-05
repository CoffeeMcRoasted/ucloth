#include <gtest/gtest.h>
#include <umath/umath.h>
#include <simulation/constraint.h>
#include <simulation/pbdsystem.h>
#include <simulation/world.h>

class Constraint_test : public ::testing::Test
{
protected:
    void SetUp() override
    {
    }
    void TearDown() override
    {
        world.clear();
    };
    ucloth::umath::Real const delta_time = 0.033;
    ucloth::simulation::World world;
};

TEST_F(Constraint_test, Distance_constraint_test)
{
    using namespace ucloth;

    umath::Real const test_distance = 1.0f;
    umath::Real const test_stiffness = 1.0f;
    world.positions = {{0.0, 0.0, 0.0}, {0.0, 0.8, 0.8}};
    world.inverse_particle_masses = {1.0f, 1.0f};
    simulation::Distance_constraint test;
    test.p1 = 0;
    test.p2 = 1;
    test.distance = test_distance;
    test.stiffness = test_stiffness;
    size_t const solver_iterations = 10;
    for (size_t i = 0; i < solver_iterations; ++i)
    {
        simulation::project_distance_constraints(
            {test}, world.inverse_particle_masses, solver_iterations, world.positions);
    }
    umath::Real const test_final_distance = umath::length(world.positions[0] - world.positions[1]);

    EXPECT_NEAR(test_final_distance, test_distance, 1e-8);
}

TEST_F(Constraint_test, Bending_constraint_test)
{
    using namespace ucloth;
    umath::Real const p_inv_mass = 1 / 0.01;
    umath::Real const test_stiffness = 0.9f;
    world.positions = {
        {0.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {1.0, 1.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {2.0, 1.0, 0.0}};
    world.velocities = {
        {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 10.0}, {0.0, 0.0, 10.0}};
    world.inverse_particle_masses = {p_inv_mass, p_inv_mass, p_inv_mass, p_inv_mass, p_inv_mass, p_inv_mass};
    world.meshes = {{{{0, 3, 1}, {1, 3, 2}, {3, 4, 2}, {2, 4, 5}}, 0, 5}};  //< 2 quads composed of 4 triangles.
    std::vector<umath::Position> position_estimates;
    simulation::PBD_system::calculate_position_estimates(
        world.positions, world.velocities, delta_time, position_estimates);
    std::vector<simulation::Bending_constraint> test_constraints;
    simulation::Bending_constraint b1;
    test_constraints.emplace_back(simulation::Bending_constraint{1, 3, 0, 2, acosf(0.0f), test_stiffness});
    // test_constraints.emplace_back(simulation::Bending_constraint{2, 0, 3, 1, acosf(0.0f), test_stiffness});
    test_constraints.emplace_back(simulation::Bending_constraint{2, 3, 1, 4, acosf(0.0f), test_stiffness});
    // test_constraints.emplace_back(simulation::Bending_constraint{4, 1, 3, 2, acosf(0.0f), test_stiffness});
    test_constraints.emplace_back(simulation::Bending_constraint{2, 4, 3, 5, acosf(0.0f), test_stiffness});
    // test_constraints.emplace_back(simulation::Bending_constraint{5, 3, 4, 2, acosf(0.0f), test_stiffness});

    size_t const solver_iterations = 100;
    for (size_t i = 0; i < solver_iterations; ++i)
    {
        simulation::project_bending_constraints(
            test_constraints, world.inverse_particle_masses, solver_iterations, position_estimates);
    }
    position_estimates;
}