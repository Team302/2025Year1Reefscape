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

#include <string>

// FRC Includes
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "units/angular_velocity.h"
#include "units/velocity.h"
#include "units/angle.h"
#include "frc/Timer.h"
#include <choreo/Choreo.h>

// 302 includes
#include "chassis/commands/TrajectoryDrive.h"
#include "utils/logging/debug/Logger.h"
#include "auton/drivePrimitives/AutonUtils.h"
#include "chassis/TankRequest.h"

TrajectoryDrive::TrajectoryDrive(
    CANDriveSubsystem *chassis) : m_chassis(chassis),
                                  m_pathName(""),
                                  m_trajectoryStates(),
                                  m_prevPose(),
                                  m_wasMoving(false),
                                  m_timer(std::make_unique<frc::Timer>()),
                                  m_whyDone("Trajectory isn't finished/Error"),
                                  m_totalTrajectoryTime(units::time::second_t(0.0))
{
    // This command requires the chassis subsystem
    AddRequirements(m_chassis);
}

void TrajectoryDrive::Initialize()
{
    m_trajectory = AutonUtils::GetDifferentialTrajectoryFromPathFile(m_pathName);
    m_trajectoryStates = m_trajectory.value().samples;

    // Reset and start the timer when the command begins
    m_timer.get()->Reset();
    m_timer.get()->Start();

    // Reset PID controllers to clear any previous state
    m_xController.Reset();
    m_yController.Reset();
}

void TrajectoryDrive::SetPath(const std::string &pathName)
{
    m_pathName = pathName;
}

void TrajectoryDrive::Execute()
{
    if (!m_trajectoryStates.empty()) // If we have a path parsed / have states to run
    {
        auto desiredState = m_trajectory.value().SampleAt(m_timer.get()->Get()).value();
        if (m_chassis != nullptr)
        {
            m_chassis->TankDrive(desiredState.vl, desiredState.vr);
        }
    }
}

bool TrajectoryDrive::IsFinished()
{
    bool isDone = false;

    // auto currentPose = m_chassis != nullptr ? m_chassis->GetPose() : frc::Pose2d();
    if (!m_trajectoryStates.empty())
    {
        // auto currentTime = m_timer.get()->Get();

        // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "current time", currentTime.value());
        // Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "total time", m_totalTrajectoryTime.value());

        // if ((currentTime) / m_totalTrajectoryTime > 0.9)
        // {

        //     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "current pose X", currentPose.X().value());
        //     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "current pose Y", currentPose.Y().value());
        //     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "current pose Rotation", currentPose.Rotation().Degrees().value());

        //     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "target pose X", m_finalState.GetPose().X().value());
        //     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "target pose Y", m_finalState.GetPose().Y().value());
        //     Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "TrajectoryDrive", "target pose Rotation", m_finalState.GetPose().Rotation().Degrees().value());

        //     isDone = IsSamePose(currentPose, m_finalState.GetPose(), m_chassisSpeeds, 10.0, 3.0, 1.5); // TO DO verify these values
        // }
        // else if (m_chassis != nullptr)
        // {
        //     isDone = m_chassis->IsSamePose();
        // }
    }
    else
    {
        m_whyDone = "No states in trajectory";
        isDone = true;
        Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "", "why done", m_whyDone);
    }

    return isDone;
}

void TrajectoryDrive::End(bool interrupted)
{
    // When the command ends (or is interrupted), stop the robot.
    m_chassis->SetControl(drive::tank::requests::TankDriveBrake{});
}

bool TrajectoryDrive::IsSamePose(frc::Pose2d currentPose, frc::Pose2d previousPose, frc::ChassisSpeeds velocity, double xyTolerance, double rotTolerance, double speedTolerance)
{
    // Detect if the two poses are the same within a tolerance
    double dCurPosX = currentPose.X().to<double>() * 100; // cm
    double dCurPosY = currentPose.Y().to<double>() * 100;
    double dPrevPosX = previousPose.X().to<double>() * 100;
    double dPrevPosY = previousPose.Y().to<double>() * 100;

    double dCurPosRot = currentPose.Rotation().Degrees().to<double>();
    double dPrevPosRot = previousPose.Rotation().Degrees().to<double>();

    dCurPosRot = dCurPosRot < 0 ? 360 + dCurPosRot : dCurPosRot;
    dPrevPosRot = dPrevPosRot < 0 ? 360 + dPrevPosRot : dPrevPosRot;

    double dDeltaX = abs(dPrevPosX - dCurPosX);
    double dDeltaY = abs(dPrevPosY - dCurPosY);
    double dDeltaRot = abs(dCurPosRot - dPrevPosRot);

    units::velocity::meters_per_second_t chassisSpeed = units::math::sqrt((velocity.vx * velocity.vx) + (velocity.vy * velocity.vy));

    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive", "deltaX", dPrevPosX - dCurPosX);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive", "deltaY", dPrevPosY - dCurPosY);
    Logger::GetLogger()->LogData(LOGGER_LEVEL::PRINT, "trajectory drive", "deltaRotation", dDeltaRot);

    //  If Position of X or Y has moved since last scan..  Using Delta X/Y
    return ((dDeltaX <= xyTolerance) && (dDeltaY <= xyTolerance) && (dDeltaRot <= rotTolerance) && (chassisSpeed.to<double>() <= speedTolerance));
}