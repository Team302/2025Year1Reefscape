// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include "chassis/CANDriveSubsystem.h"

class AutoCommand : public frc2::CommandHelper<frc2::Command, AutoCommand> {
 public:
  AutoCommand(CANDriveSubsystem *driveSubsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  CANDriveSubsystem *driveSubsystem;

  frc::Timer timer;
  double seconds = 1;
};