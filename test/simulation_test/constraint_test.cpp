#include <gtest/gtest.h>
#include <simulation/constraint.h>
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
    ucloth::simulation::World world;
};

TEST(Constraint_test, Distance_constraint_test)
{
    using namespace ucloth::simulation;

    Distance_constraint test;
}