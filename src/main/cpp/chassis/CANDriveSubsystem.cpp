// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CANDriveSubsystem.h"
#include "TankRequest.h"

using namespace DriveConstants;
// Singleton instance
CANDriveSubsystem *CANDriveSubsystem::m_instance = nullptr;

CANDriveSubsystem *CANDriveSubsystem::GetInstance()
{
  if (m_instance == nullptr)
  {
    m_instance = new CANDriveSubsystem();
  }
  return m_instance;
}

// class to drive the robot over CAN
CANDriveSubsystem::CANDriveSubsystem()
    : leftLeader{LEFT_LEADER_ID, rev::spark::SparkMax::MotorType::kBrushed},
      leftFollower{LEFT_FOLLOWER_ID, rev::spark::SparkMax::MotorType::kBrushed},
      rightLeader{RIGHT_LEADER_ID, rev::spark::SparkMax::MotorType::kBrushed},
      rightFollower{RIGHT_FOLLOWER_ID,
                    rev::spark::SparkMax::MotorType::kBrushed}
{
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
void CANDriveSubsystem::ArcadeDrive(double xSpeed, double zRotation)
{
  drive.ArcadeDrive(xSpeed, zRotation);
}

bool CANDriveSubsystem::IsSamePose()
{
  bool isCurrentlyStopped = GetPose().Translation().Distance(m_prevPose.Translation()) < m_distanceThreshold;

  if (isCurrentlyStopped)
  {
    if (!m_debounceTimer.IsRunning())
    {
      m_debounceTimer.Start();
    }
  }
  else
  {
    m_debounceTimer.Stop();
    m_debounceTimer.Reset();
  }

  return m_debounceTimer.HasElapsed(m_samePoseTime);
}

// Overloaded SetControl for FieldCentric
void CANDriveSubsystem::SetControl(const drive::tank::requests::FieldCentric &request)
{
  // Apply FieldCentric control logic
  leftLeader.Set(request.VelocityForward.to<double>());
  rightLeader.Set(request.VelocityForward.to<double>());
}

// Overloaded SetControl for RobotCentric
void CANDriveSubsystem::SetControl(const drive::tank::requests::RobotCentric &request)
{
  // Apply RobotCentric control logic
  leftLeader.Set(request.VelocityForward.to<double>());
  rightLeader.Set(request.VelocityForward.to<double>());
}

// Overloaded SetControl for TankDriveBrake
void CANDriveSubsystem::SetControl(const drive::tank::requests::TankDriveBrake &request)
{
  // Apply TankDriveBrake control logic (stop the motors)
  leftLeader.Set(0.0);
  rightLeader.Set(0.0);
}

// Overloaded SetControl for Idle
void CANDriveSubsystem::SetControl(const drive::tank::requests::Idle &request)
{
  // Apply Idle control logic (do nothing)
}

// Overloaded SetControl for FieldCentricFacingAngle
void CANDriveSubsystem::SetControl(const drive::tank::requests::FieldCentricFacingAngle &request)
{
  // Placeholder: treat like FieldCentric using VelocityForward only (preserve current style)
  leftLeader.Set(request.VelocityForward.to<double>());
  rightLeader.Set(request.VelocityForward.to<double>());
}

void CANDriveSubsystem::ResetPose(frc::Pose2d pose)
{
  // Placeholder: reset pose logic
  m_prevPose = pose;
}
void CANDriveSubsystem::ResetSamePose()
{
  m_debounceTimer.Stop();
  m_debounceTimer.Reset();
}
void CANDriveSubsystem::AddVisionMeasurement(const frc::Pose2d &visionPose, units::second_t timeStamp, const std::array<double, 3> &stdDevs)
{
  // Placeholder: add vision measurement logic
}
void CANDriveSubsystem::AddVisionMeasurement(const frc::Pose2d &visionPose, units::second_t timeStamp)
{
  // Placeholder: add vision measurement logic
}
void CANDriveSubsystem::SeedFieldCentric()
{
  // Placeholder: seed field centric logic
}
double CANDriveSubsystem::GetRotationRateDegreesPerSecond()
{
  // Placeholder: return rotation rate logic
  return 0.0;
}