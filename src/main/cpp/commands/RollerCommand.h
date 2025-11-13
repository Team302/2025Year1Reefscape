// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>

#include <utility>

#include "subsystems/CANRollerSubsystem.h"

class RollerCommand : public frc2::CommandHelper<frc2::Command, RollerCommand> {
 public:
  RollerCommand(CANRollerSubsystem *rollerSubsystem,
                std::function<double()> forward,
                std::function<double()> reverse);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  CANRollerSubsystem *rollerSubsystem;
  std::function<double()> forward;
  std::function<double()> reverse;
};