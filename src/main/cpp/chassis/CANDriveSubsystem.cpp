// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "CANDriveSubsystem.h"
#include "utils/logging/debug/Logger.h"

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
  sparkConfig.Inverted(true);
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
  sparkConfig.Inverted(false);
  leftLeader.Configure(sparkConfig,
                       rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                       rev::spark::SparkBase::PersistMode::kPersistParameters);

  m_odometryTimer.Start();
  m_lastTime = m_odometryTimer.Get();
};

void CANDriveSubsystem::Periodic()
{
}

void CANDriveSubsystem::SimulationPeriodic() {}

// sets the speed of the drive motors
void CANDriveSubsystem::ArcadeDrive(double xSpeed, double zRotation)
{
  UpdateOdometry(xSpeed, zRotation);
  drive.ArcadeDrive(xSpeed, zRotation);
}

// sets the speed of the drive motors
void CANDriveSubsystem::TankDrive(units::velocity::meters_per_second_t vL, units::velocity::meters_per_second_t vR)
{
  UpdateOdometry(vL, vR);
  auto leftSpeed = (vL / DriveConstants::MAX_DRIVE_SPEED).value();
  auto rightSpeed = (vR / DriveConstants::MAX_DRIVE_SPEED).value();
  drive.TankDrive(leftSpeed, rightSpeed);
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

void CANDriveSubsystem::ResetPose(frc::Pose2d pose)
{
  m_currentPose = pose;
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

double CANDriveSubsystem::GetRotationRateDegreesPerSecond()
{
  return 0.0;
}

void CANDriveSubsystem::UpdateOdometry(double xSpeed, double zRotation)
{
  double leftPercent = xSpeed + zRotation;
  double rightPercent = xSpeed - zRotation;

  leftPercent = std::clamp(leftPercent, -1.0, 1.0);
  rightPercent = std::clamp(rightPercent, -1.0, 1.0);

  auto vL = leftPercent * DriveConstants::MAX_DRIVE_SPEED;
  auto vR = rightPercent * DriveConstants::MAX_DRIVE_SPEED;

  UpdateOdometry(vL, vR);
}

void CANDriveSubsystem::UpdateOdometry(units::velocity::meters_per_second_t vL, units::velocity::meters_per_second_t vR)
{
  m_prevPose = m_currentPose;

  units::second_t currentTime = m_odometryTimer.Get();
  units::second_t dt = currentTime - m_lastTime;
  m_lastTime = currentTime;

  frc::DifferentialDriveWheelSpeeds wheelSpeeds{vL, vR};
  frc::ChassisSpeeds chassisSpeeds = m_kinematics.ToChassisSpeeds(wheelSpeeds);

  frc::Twist2d twist{
      chassisSpeeds.vx * dt,
      chassisSpeeds.vy * dt,
      chassisSpeeds.omega * dt};

  m_currentPose = m_currentPose.Exp(twist);

  Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "CANDriveSubsystem", "Current Pose X", m_currentPose.X().value());
  Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "CANDriveSubsystem", "Current Pose Y", m_currentPose.Y().value());
}