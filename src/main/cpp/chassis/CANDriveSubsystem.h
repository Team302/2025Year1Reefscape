// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"
#include <frc/RobotController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <frc/geometry/Pose2d.h>

class CANDriveSubsystem : public frc2::SubsystemBase {
 public:

  static CANDriveSubsystem *GetInstance();
  void Periodic() override;

  void SimulationPeriodic() override;

  void ArcadeDrive(double xSpeed, double zRotation);

  bool IsSamePose();

  frc::Pose2d GetPose() {return frc::Pose2d();} //TODO: fix this;

  void SetControl(){};

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
