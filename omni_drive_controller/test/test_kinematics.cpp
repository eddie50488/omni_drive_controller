#include <gmock/gmock.h>
#include <vector>
#include "omni_drive_controller/kinematics.hpp"
#include "omni_drive_controller/types.hpp"

constexpr double TOLERANCE = 1e-3;

class TestOmnidirectionalControllersKinematics : public ::testing::Test
{
protected:
  omni_drive_controller::Kinematics kinematics_;
};

TEST_F(TestOmnidirectionalControllersKinematics, TestInverseKinematics)
{
  std::vector<double> angular_velocities;
  omni_drive_controller::RobotVelocity vel;
  kinematics_.setRobotParams({0.0505});

  vel = {1, 0, 0};
  angular_velocities = kinematics_.getWheelsAngularVelocities(vel);

  ASSERT_GT(angular_velocities.size(), 0.0);
  EXPECT_NEAR(angular_velocities[0], 1, TOLERANCE);
  EXPECT_NEAR(angular_velocities[1], 1, TOLERANCE);
  EXPECT_NEAR(angular_velocities[2], 0, TOLERANCE);
  EXPECT_NEAR(angular_velocities[3], 0, TOLERANCE);

  vel = {0, 1, 0};
  angular_velocities = kinematics_.getWheelsAngularVelocities(vel);

  ASSERT_GT(angular_velocities.size(), 0.0);
  EXPECT_NEAR(angular_velocities[0], 0, TOLERANCE);
  EXPECT_NEAR(angular_velocities[1], 0, TOLERANCE);
  EXPECT_NEAR(angular_velocities[2], 1, TOLERANCE);
  EXPECT_NEAR(angular_velocities[3], 1, TOLERANCE);

  vel = {0, 0, 1};
  angular_velocities = kinematics_.getWheelsAngularVelocities(vel);

  ASSERT_GT(angular_velocities.size(), 0.0);
  EXPECT_NEAR(angular_velocities[0], 1, TOLERANCE);
  EXPECT_NEAR(angular_velocities[1], 1, TOLERANCE);
  EXPECT_NEAR(angular_velocities[2], 1, TOLERANCE);
  EXPECT_NEAR(angular_velocities[3], 1, TOLERANCE);
}
