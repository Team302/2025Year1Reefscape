
//====================================================================================================================================================
// Copyright 2025 Lake Orion Robotics FIRST Team 302
//
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
// OR OTHER DEALINGS IN THE SOFTWARE.
//====================================================================================================================================================
#include "TankContainer.h"
#include "commands/DriveToTarget.h"
#include "commands/VisionDrive.h"
#include "state/RobotState.h"
#include "commands/TrajectoryDrive.h"
#include "commands/TeleopRobotDrive.h"
#include "frc2/command/Commands.h"
#include "frc2/command/button/RobotModeTriggers.h"
#include "frc2/command/ProxyCommand.h"

TankContainer *TankContainer::m_instance = nullptr;

TankContainer *TankContainer::GetInstance()
{
    if (TankContainer::m_instance == nullptr)
    {
        TankContainer::m_instance = new TankContainer();
    }
    return TankContainer::m_instance;
}

TankContainer::TankContainer()
    : m_chassis(CANDriveSubsystem::GetInstance()),
      m_maxSpeed(DriveConstants::MAX_DRIVE_SPEED),
      m_robotDrive(std::make_unique<TeleopRobotDrive>(m_chassis, TeleopControl::GetInstance(), m_maxSpeed, m_maxAngularRate)),
      m_driveToCoralStationSidewall(std::make_unique<DriveToTarget>(m_chassis, DragonTargetFinderTarget::CLOSEST_CORAL_STATION_SIDWALL_SIDE)),
      m_driveToCoralStationAlliance(std::make_unique<DriveToTarget>(m_chassis, DragonTargetFinderTarget::CLOSEST_CORAL_STATION_ALLIANCE_SIDE)),
      m_driveToCoralRightBranch(std::make_unique<DriveToTarget>(m_chassis, DragonTargetFinderTarget::CLOSEST_RIGHT_REEF_BRANCH)),
      m_driveToCoralLeftBranch(std::make_unique<DriveToTarget>(m_chassis, DragonTargetFinderTarget::CLOSEST_LEFT_REEF_BRANCH)),
      m_driveToBarge(std::make_unique<DriveToTarget>(m_chassis, DragonTargetFinderTarget::BARGE)),
      m_driveToLeftCage(std::make_unique<DriveToTarget>(m_chassis, DragonTargetFinderTarget::LEFT_CAGE)),
      m_driveToRightCage(std::make_unique<DriveToTarget>(m_chassis, DragonTargetFinderTarget::RIGHT_CAGE)),
      m_driveToCenterCage(std::make_unique<DriveToTarget>(m_chassis, DragonTargetFinderTarget::CENTER_CAGE)),
      m_driveToAlgae(std::make_unique<VisionDrive>(m_chassis, TeleopControl::GetInstance(), m_maxSpeed, m_maxAngularRate)),
      m_trajectoryDrive(std::make_unique<TrajectoryDrive>(m_chassis))
{

    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus_Int);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredCoralSide_Int);

    if (m_chassis != nullptr)
    {
        ConfigureBindings();
    }
}

void TankContainer::ConfigureBindings()
{
    auto controller = TeleopControl::GetInstance();

    CreateStandardDriveCommands(controller);
    CreateReefscapeDriveToCommands(controller);
}
void TankContainer::CreateStandardDriveCommands(TeleopControl *controller)
{
    auto isResetYawSelected = controller->GetCommandTrigger(TeleopControlFunctions::RESET_POSITION);
    auto isRobotOriented = controller->GetCommandTrigger(TeleopControlFunctions::ROBOT_ORIENTED_DRIVE);

    if (m_chassis != nullptr)
    {
        m_chassis->SetDefaultCommand(std::move(m_robotDrive));
    }
}

void TankContainer::CreateReefscapeDriveToCommands(TeleopControl *controller)
{
    auto driveToRightReefBranch = controller->GetCommandTrigger(TeleopControlFunctions::AUTO_ALIGN_RIGHT);
    auto driveToLeftReefBranch = controller->GetCommandTrigger(TeleopControlFunctions::AUTO_ALIGN_LEFT);
    auto driveToCoralStation = controller->GetCommandTrigger(TeleopControlFunctions::AUTO_ALIGN_HUMAN_PLAYER_STATION);
    auto driveToBarge = controller->GetCommandTrigger(TeleopControlFunctions::AUTO_ALIGN_BARGE);
    auto driveToAglee = controller->GetCommandTrigger(TeleopControlFunctions::AUTO_ALIGN_ALGAE);

    driveToBarge.WhileTrue(frc2::ProxyCommand(m_driveToBarge.get()).ToPtr());

    driveToAglee.WhileTrue(frc2::cmd::DeferredProxy([this]() -> frc2::CommandPtr
                                                    {
    if (!m_climbMode) {
        return frc2::ProxyCommand(m_driveToAlgae.get()).ToPtr();
    } else {
        return frc2::cmd::None(); // No command if in climb mode
    } }));

    driveToCoralStation.WhileTrue(frc2::cmd::DeferredProxy([this]() -> frc2::CommandPtr
                                                           {
    if (m_climbMode) {
        return frc2::ProxyCommand(m_driveToCenterCage.get()).ToPtr();
    }
    else
    {
        if (m_desiredCoralSide == RobotStateChanges::DesiredCoralSide::AllianceWall)
        {
            return frc2::ProxyCommand(m_driveToCoralStationAlliance.get()).ToPtr();
        }   
        else
        {
            return frc2::ProxyCommand(m_driveToCoralStationSidewall.get()).ToPtr();
        } } }));

    driveToLeftReefBranch.WhileTrue(frc2::cmd::DeferredProxy([this]() -> frc2::CommandPtr
                                                             {
    if (m_climbMode) {
        return frc2::ProxyCommand(m_driveToLeftCage.get()).ToPtr();
    }
    else
    {
        return frc2::ProxyCommand(m_driveToCoralLeftBranch.get()).ToPtr();
    } }));

    driveToRightReefBranch.WhileTrue(frc2::cmd::DeferredProxy([this]() -> frc2::CommandPtr
                                                              {
    if (m_climbMode) {
        return frc2::ProxyCommand(m_driveToRightCage.get()).ToPtr();
    }
    else
    {
        return frc2::ProxyCommand(m_driveToCoralRightBranch.get()).ToPtr();
    } }));
}