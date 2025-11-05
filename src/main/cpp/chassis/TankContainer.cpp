#include "TankContainer.h"
#include "commands/ChassisConfigMgr.h"
#include "commands/TeleopFieldDrive.h"
#include "commands/TeleopRobotDrive.h"
#include "commands/DriveToTarget.h"
#include "commands/VisionDrive.h"
#include "RobotState.h"
#include "commands/TrajectoryDrive.h"

TankContainer *TankContainer::m_instance = nullptr;

TankContainer *TankContainer::GetInstance() {
    if (TankContainer::m_instance == nullptr) {
        TankContainer::m_instance = new TankContainer();
    }
    return TankContainer::m_instance;
}

TankContainer::TankContainer() 
    : m_chassis(CANDriveSubsystem::GetInstance()),
      m_maxSpeed(ChassisConfigMgr::GetInstance()->GetMaxSpeed()),
      m_fieldDrive(std::make_unique<TeleopFieldDrive>(m_chassis, TeleopControl::GetInstance(), m_maxSpeed, m_maxAngularRate)),
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
      m_trajectoryDrive(std::make_unique<TrajectoryDrive>(m_chassis)) {

    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::ClimbModeStatus_Int);
    RobotState::GetInstance()->RegisterForStateChanges(this, RobotStateChanges::StateChange::DesiredCoralSide_Int);

    if (m_chassis != nullptr) {
        ConfigureBindings();
    }
}