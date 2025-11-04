// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>

#include "commands/AutoCommand.h"
#include "commands/DriveCommand.h"
#include "commands/RollerCommand.h"

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Set the default command for the drivetrain to use the DriveCommand to drive
  // the robot with the left Y and right X axes of the gamepad
  driveSubsystem.SetDefaultCommand(DriveCommand(
      &driveSubsystem, [this] { return -driverController.GetLeftY(); },
      [this] { return -driverController.GetRightX(); }));

  // Set the default command for the roller to be analog control mapped to the
  // trigger axes of the 2nd gamepad
  rollerSubsystem.SetDefaultCommand(RollerCommand(
      &rollerSubsystem,
      [this] { return operatorController.GetRightTriggerAxis(); },
      [this] { return operatorController.GetLeftTriggerAxis(); }));

  // While the A button of the 2nd gamepad is held, run the roller at a fixed
  // speed
  //Before
  operatorController.A().WhileTrue(
      RollerCommand(
          &rollerSubsystem,
          [this] { return RollerConstants::ROLLER_MOTOR_EJECT_SPEED; },
          [this] { return 0; })
          .WithName("Fixed speed"));
  //After
  operatorController.X().WhileTrue(
      RollerCommand(
          &rollerSubsystem,
          [this] { return RollerConstants::ROLLER_MOTOR_EJECT_SPEED; },
          [this] { return 0; })
          .WithName("Fixed speed"));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return AutoCommand(&driveSubsystem).ToPtr();
}
