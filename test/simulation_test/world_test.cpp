#include <gtest/gtest.h>
#include <simulation/world.h>

TEST(UClothWorldTest, HelloTest)
{
    EXPECT_EQ(true, true);
}

TEST(UClothWorldTest, CreateWorld)
{
    using namespace ucloth::simulation;
    World world;
    EXPECT_TRUE(world.positions.empty());
    EXPECT_TRUE(world.velocities.empty());
    EXPECT_TRUE(world.inverse_particle_masses.empty());
    EXPECT_TRUE(world.bending_constraints.empty());
    EXPECT_TRUE(world.distance_constraints.empty());
    EXPECT_TRUE(world.accelerations.empty());
    EXPECT_TRUE(world.meshes.empty());
}
