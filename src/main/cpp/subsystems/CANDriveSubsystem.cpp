// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <subsystems/CANDriveSubsystem.h>

using namespace DriveConstants;

// class to drive the robot over CAN
CANDriveSubsystem::CANDriveSubsystem()
    : leftLeader{LEFT_LEADER_ID, rev::spark::SparkMax::MotorType::kBrushed},
      leftFollower{LEFT_FOLLOWER_ID, rev::spark::SparkMax::MotorType::kBrushed},
      rightLeader{RIGHT_LEADER_ID, rev::spark::SparkMax::MotorType::kBrushed},
      rightFollower{RIGHT_FOLLOWER_ID,
                    rev::spark::SparkMax::MotorType::kBrushed} {
  // set can timeout, because this project only configures on startup and
  // doesn't query any parameters, timeouts can be very long. Projects which
  // update configuration or retrieve parameters during runtime should use much
  // shorter timeouts such as the default
  leftLeader.SetCANTimeout(250);
  leftFollower.SetCANTimeout(250);
  rightLeader.SetCANTimeout(250);
  rightFollower.SetCANTimeout(250);

  // Create a config object for the SPARK MAXs
  rev::spark::SparkBaseConfig sparkConfig;
  // enable voltage compensation set to 12V. This helps normalize performance
  // over a wider range of battery charge at the cost of some peak performance
  // with fully charged batteries.
  sparkConfig.VoltageCompensation(12.0);
  // enable current limit
  sparkConfig.SmartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT);

  // Set followers by setting the configuration to follow then configuring the
  // appropriate Spark MAX
  sparkConfig.Follow(leftLeader);
  leftFollower.Configure(
      sparkConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
      rev::spark::SparkBase::PersistMode::kPersistParameters);
  sparkConfig.Follow(rightLeader);
  rightFollower.Configure(
      sparkConfig, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
      rev::spark::SparkBase::PersistMode::kPersistParameters);

  // clear follower flag for setting up leaders
  sparkConfig.DisableFollowerMode();

  // configure right leader first, then set inverted flag and configure left
  // leader
  rightLeader.Configure(sparkConfig,
                        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                        rev::spark::SparkBase::PersistMode::kPersistParameters);
  sparkConfig.Inverted(true);
  leftLeader.Configure(sparkConfig,
                       rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                       rev::spark::SparkBase::PersistMode::kPersistParameters);
};

void CANDriveSubsystem::Periodic() {}

void CANDriveSubsystem::SimulationPeriodic() {}

// sets the speed of the drive motors
void CANDriveSubsystem::ArcadeDrive(double xSpeed, double zRotation) {
  drive.ArcadeDrive(xSpeed, zRotation);
}