// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/RobotController.h>
#include <subsystems/CANRollerSubsystem.h>

using namespace RollerConstants;

// class to run the rollers over CAN
CANRollerSubsystem::CANRollerSubsystem()
    : rollerMotor{ROLLER_MOTOR_ID, rev::spark::SparkMax::MotorType::kBrushed} {
  rev::spark::SparkBaseConfig sparkConfig;

  // set can timeout, because this project only configures on startup and
  // doesn't query any parameters, timeouts can be very long. Projects which
  // update configuration or retrieve parameters during runtime should use much
  // shorter timeouts such as the default
  rollerMotor.SetCANTimeout(250);

  // voltage compensation keeps the roller behaviour more consistent over a
  // range of battery voltages
  sparkConfig.VoltageCompensation(ROLLER_MOTOR_VOLTAGE_COMP);
  sparkConfig.SmartCurrentLimit(ROLLER_MOTOR_CURRENT_LIMIT);
  rollerMotor.Configure(sparkConfig,
                        rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                        rev::spark::SparkBase::PersistMode::kPersistParameters);
};

void CANRollerSubsystem::Periodic() {}

void CANRollerSubsystem::SimulationPeriodic() {}

// sets the speed of the roller motor
void CANRollerSubsystem::RunRoller(double forward, double reverse) {
  rollerMotor.Set(forward - reverse);
}