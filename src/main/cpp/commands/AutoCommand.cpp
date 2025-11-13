// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "AutoCommand.h"

#include <utility>

// Command to drive the robot at 1/2 power for one second in autonomous mode.
AutoCommand::AutoCommand(CANDriveSubsystem *driveSubsystem)
    : driveSubsystem(driveSubsystem) {
  AddRequirements(driveSubsystem);
}

void AutoCommand::Initialize() {
  // start timer when command is scheduled
  timer.Restart();
}

void AutoCommand::Execute() {
  // run the drive train at 50% power forward (called every ~every 20ms)
  driveSubsystem->ArcadeDrive(0.5, 0.0);
}

void AutoCommand::End(bool interrupted) {
  // stop the timer and drive train
  timer.Stop();
  driveSubsystem->ArcadeDrive(0.0, 0.0);
}

bool AutoCommand::IsFinished() {
  // the command finishes when the timer exceeds the number of seconds
  return timer.Get().value() >= seconds;
}