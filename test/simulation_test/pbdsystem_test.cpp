#include <gtest/gtest.h>
#include <simulation/pbdsystem.h>
#include <vector>

class PBD_system_test : public ::testing::Test
{
protected:
    void SetUp() override
    {
    }
    void TearDown() override
    {
    }
};

TEST(PBD_system_test, External_acceleration_test)
{
    using namespace ucloth::simulation;
    using namespace ucloth::umath;

    // Gravity
    std::vector<Vec3> accelerations = {{0.0f, 0.0f, -9.8f}};
    // Single particle
    std::vector<Vec3> velocities = {{0.0f, 0.0f, 0.0f}};
    // Simulation time
    Real dt = 0.1;

    PBD_system::apply_external_accelerations(accelerations, dt, velocities);
    EXPECT_EQ(velocities.front().x, 0.0f);
    EXPECT_EQ(velocities.front().y, 0.0f);
    EXPECT_NEAR(velocities.front().z, -9.8f * 0.1, 1E-9);
}