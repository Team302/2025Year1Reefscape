// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <utility>

#include "subsystems/CANDriveSubsystem.h"

class DriveCommand : public frc2::CommandHelper<frc2::Command, DriveCommand> {
 public:
  DriveCommand(CANDriveSubsystem *driveSubsystem,
               std::function<double()> xSpeed,
               std::function<double()> zRotation);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  CANDriveSubsystem *driveSubsystem;
  std::function<double()> xSpeed;
  std::function<double()> zRotation;
};