// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <units/velocity.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants
{
    inline constexpr int LEFT_LEADER_ID = 1;
    inline constexpr int LEFT_FOLLOWER_ID = 2;
    inline constexpr int RIGHT_LEADER_ID = 3;
    inline constexpr int RIGHT_FOLLOWER_ID = 4;

    inline constexpr int DRIVE_MOTOR_CURRENT_LIMIT = 60;
    inline constexpr units::meters_per_second_t MAX_DRIVE_SPEED = 4_mps;
} // namespace DriveConstants

namespace RollerConstants
{
    inline constexpr int ROLLER_MOTOR_ID = 5;
    inline constexpr int ROLLER_MOTOR_CURRENT_LIMIT = 60;
    inline constexpr double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    inline constexpr double ROLLER_MOTOR_EJECT_SPEED = 0.44;
} // namespace RollerConstants

namespace OperatorConstants
{
    inline constexpr int DRIVER_CONTROLLER_PORT = 0;
    inline constexpr int OPERATOR_CONTROLLER_PORT = 1;
} // namespace OperatorConstants
