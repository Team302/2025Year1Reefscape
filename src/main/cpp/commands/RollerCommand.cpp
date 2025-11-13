// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RollerCommand.h"

#include <utility>

RollerCommand::RollerCommand(CANRollerSubsystem *rollerSubsystem,
                             std::function<double()> forward,
                             std::function<double()> reverse)
    : rollerSubsystem(rollerSubsystem), forward(forward), reverse(reverse) {
  AddRequirements(rollerSubsystem);
}

void RollerCommand::Initialize() {}

void RollerCommand::Execute() {
  // run the roller at the speed set by the lambda functions
  rollerSubsystem->RunRoller(forward(), reverse());
}

void RollerCommand::End(bool interrupted) {}

bool RollerCommand::IsFinished() { return false; }