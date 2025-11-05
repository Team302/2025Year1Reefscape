#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <units/velocity.h>
#include <units/angular_velocity.h>
#include "commands/TrajectoryDrive.h"
#include "CANDriveSubsystem.h"

class TankContainer : CANDriveSubsystem
{
public:
    static TankContainer *GetInstance();

    frc2::Command *GetDriveToCoralStationCommand() { return m_driveToCoralStationSidewall.get(); }
    frc2::Command *GetDriveToCoralLeftBranchCommand() { return m_driveToCoralLeftBranch.get(); }
    frc2::Command *GetDriveToCoralRightBranchCommand() { return m_driveToCoralRightBranch.get(); }
    frc2::Command *GetDriveToBargeCommand() { return m_driveToBarge.get(); }
    TrajectoryDrive *GetTrajectoryDriveCommand() { return m_trajectoryDrive.get(); }

private:
    TankContainer();
    virtual ~TankContainer() = default;
    static TankContainer *m_instance;

    CANDriveSubsystem *m_chassis;

    units::meters_per_second_t m_maxSpeed;
    static constexpr units::radians_per_second_t m_maxAngularRate{1.5_tps};

    frc2::CommandPtr m_fieldDrive;
    frc2::CommandPtr m_robotDrive;
    frc2::CommandPtr m_driveToCoralStationSidewall;
    frc2::CommandPtr m_driveToCoralStationAlliance;
    frc2::CommandPtr m_driveToCoralRightBranch;
    frc2::CommandPtr m_driveToCoralLeftBranch;
    frc2::CommandPtr m_driveToBarge;
    frc2::CommandPtr m_driveToLeftCage;
    frc2::CommandPtr m_driveToRightCage;
    frc2::CommandPtr m_driveToCenterCage;
    frc2::CommandPtr m_driveToAlgae;
};