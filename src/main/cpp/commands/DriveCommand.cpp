// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "DriveCommand.h"

// Command to drive the robot with joystick inputs
DriveCommand::DriveCommand(CANDriveSubsystem *driveSubsystem,
                           std::function<double()> xSpeed,
                           std::function<double()> zRotation)
    : driveSubsystem(driveSubsystem), xSpeed(xSpeed), zRotation(zRotation) {
  AddRequirements(driveSubsystem);
}

void DriveCommand::Initialize() {}

void DriveCommand::Execute() {
  // run drive train with specified speed and rotation lambda functions
  driveSubsystem->ArcadeDrive(xSpeed(), zRotation());
}

void DriveCommand::End(bool interrupted) {}

bool DriveCommand::IsFinished() { return false; }