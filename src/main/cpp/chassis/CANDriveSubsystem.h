// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"
#include <frc/RobotController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <frc/geometry/Pose2d.h>
#include "TankRequest.h" // Include the TankRequest header for request types
#include "units/velocity.h"

class CANDriveSubsystem : public frc2::SubsystemBase
{
public:
  static CANDriveSubsystem *GetInstance();
  void Periodic() override;
  void SimulationPeriodic() override;
  void ArcadeDrive(double xSpeed, double zRotation);
  void TankDrive(units::velocity::meters_per_second_t vL, units::velocity::meters_per_second_t vR);
  bool IsSamePose();
  frc::Pose2d GetPose() { return frc::Pose2d(); } // TODO: fix this;

  // Overloaded SetControl functions for different request types
  void SetControl(const drive::tank::requests::FieldCentric &request);
  void SetControl(const drive::tank::requests::RobotCentric &request);
  void SetControl(const drive::tank::requests::TankDriveBrake &request);
  void SetControl(const drive::tank::requests::Idle &request);
  void SetControl(const drive::tank::requests::FieldCentricFacingAngle &request);

  // ApplyRequest helpers (lvalue-producing supplier)
  template <typename RequestSupplier>
    requires std::is_lvalue_reference_v<std::invoke_result_t<RequestSupplier>> &&
             requires(RequestSupplier rs, CANDriveSubsystem &drive) { drive.SetControl(rs()); }
  frc2::CommandPtr ApplyRequest(RequestSupplier request)
  {
    return frc2::cmd::Run([this, request = std::move(request)]
                          { SetControl(request()); }, {this});
  }

  // ApplyRequest helpers (rvalue / value-producing supplier)
  template <typename RequestSupplier>
    requires std::negation_v<std::is_lvalue_reference<std::invoke_result_t<RequestSupplier>>> &&
             requires(RequestSupplier rs, CANDriveSubsystem &drive) { drive.SetControl(rs()); }
  frc2::CommandPtr ApplyRequest(RequestSupplier request)
  {
    return frc2::cmd::Run([this, request = std::move(request)]
                          { SetControl(request()); }, {this});
  }

  void ResetPose(frc::Pose2d pose);
  void ResetSamePose();
  void AddVisionMeasurement(const frc::Pose2d &visionPose, units::second_t timeStamp, const std::array<double, 3> &stdDevs);
  void AddVisionMeasurement(const frc::Pose2d &visionPose, units::second_t timeStamp);

  void SeedFieldCentric();

  double GetRotationRateDegreesPerSecond();

private:
  CANDriveSubsystem();

  rev::spark::SparkMax leftLeader;
  rev::spark::SparkMax leftFollower;
  rev::spark::SparkMax rightLeader;
  rev::spark::SparkMax rightFollower;

  frc::DifferentialDrive drive{leftLeader, rightLeader};
  static CANDriveSubsystem *m_instance;
  frc::Timer m_debounceTimer;
  const units::time::second_t m_samePoseTime = 0.5_s;
  const units::length::inch_t m_distanceThreshold{0.25};
  frc::Pose2d m_prevPose;
};
