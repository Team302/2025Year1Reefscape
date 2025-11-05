#pragma once

#include "ctre/phoenix6/SignalLogger.hpp"

#include <frc/DriverStation.h>
#include <frc/Notifier.h>
#include "frc2/command/CommandPtr.h"
#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/Timer.h>


using namespace ctre::phoenix6;

namespace subsystems
{

    /**
     * \brief Class that extends the Phoenix 6 SwerveDrivetrain class and implements
     * Subsystem so it can easily be used in command-based projects.
     */
    class CommandDrivetrain : public frc2::SubsystemBase
    {
        static constexpr units::second_t kSimLoopPeriod = 5_ms;
        std::unique_ptr<frc::Notifier> m_simNotifier;
        units::second_t m_lastSimTime;

        /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
        static constexpr frc::Rotation2d kBlueAlliancePerspectiveRotation{0_deg};
        /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
        static constexpr frc::Rotation2d kRedAlliancePerspectiveRotation{180_deg};
        /* Keep track if we've ever applied the operator perspective before or not */
        bool m_hasAppliedOperatorPerspective = false;


    public:
        /**
         * \brief Constructs a CTRE SwerveDrivetrain using the specified constants.
         *
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them
         * through getters in the classes.
         *
         * \param drivetrainConstants Drivetrain-wide constants for the swerve drive
         * \param modules             Constants for each specific module
         */
        template <std::same_as<SwerveModuleConstants>... ModuleConstants>
        CommandSwerveDrivetrain(swerve::SwerveDrivetrainConstants const &driveTrainConstants, ModuleConstants const &...modules) : TunerSwerveDrivetrain{driveTrainConstants, modules...}
        {
            if (utils::IsSimulation())
            {
                StartSimThread();
            }
        }

        /**
         * \brief Constructs a CTRE SwerveDrivetrain using the specified constants.
         *
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them
         * through getters in the classes.
         *
         * \param driveTrainConstants        Drivetrain-wide constants for the swerve drive
         * \param odometryUpdateFrequency    The frequency to run the odometry loop. If
         *                                   unspecified or set to 0 Hz, this is 250 Hz on
         *                                   CAN FD, and 100 Hz on CAN 2.0.
         * \param modules                    Constants for each specific module
         */
        template <std::same_as<SwerveModuleConstants>... ModuleConstants>
        CommandSwerveDrivetrain(
            swerve::SwerveDrivetrainConstants const &driveTrainConstants,
            units::hertz_t odometryUpdateFrequency,
            ModuleConstants const &...modules) : TunerSwerveDrivetrain{driveTrainConstants, odometryUpdateFrequency, modules...}
        {
            if (utils::IsSimulation())
            {
                StartSimThread();
            }
        }

        /**
         * \brief Constructs a CTRE SwerveDrivetrain using the specified constants.
         *
         * This constructs the underlying hardware devices, so users should not construct
         * the devices themselves. If they need the devices, they can access them
         * through getters in the classes.
         *
         * \param driveTrainConstants        Drivetrain-wide constants for the swerve drive
         * \param odometryUpdateFrequency    The frequency to run the odometry loop. If
         *                                   unspecified or set to 0 Hz, this is 250 Hz on
         *                                   CAN FD, and 100 Hz on CAN 2.0.
         * \param odometryStandardDeviation  The standard deviation for odometry calculation
         * \param visionStandardDeviation    The standard deviation for vision calculation
         * \param modules                    Constants for each specific module
         */
        template <std::same_as<SwerveModuleConstants>... ModuleConstants>
        CommandSwerveDrivetrain(
            swerve::SwerveDrivetrainConstants const &driveTrainConstants,
            units::hertz_t odometryUpdateFrequency,
            std::array<double, 3> const &odometryStandardDeviation,
            std::array<double, 3> const &visionStandardDeviation,
            ModuleConstants const &...modules) : TunerSwerveDrivetrain{
                                                     driveTrainConstants, odometryUpdateFrequency,
                                                     odometryStandardDeviation, visionStandardDeviation, modules...}
        {
            if (utils::IsSimulation())
            {
                StartSimThread();
            }
        }

        /**
         * \brief Returns a command that applies the specified control request to this swervedrivetrain->
         *
         * This captures the returned swerve request by reference, so it must live
         * for at least as long as thedrivetrain-> This can be done by storing the
         * request as a member variable of your drivetrain subsystem or robot.
         *
         * \param request Function returning the request to apply
         * \returns Command to run
         */
        template <typename RequestSupplier>
            requires std::is_lvalue_reference_v<std::invoke_result_t<RequestSupplier>> &&
                     requires(RequestSupplier req, TunerSwerveDrivetrain &drive) { drive.SetControl(req()); }
        frc2::CommandPtr ApplyRequest(RequestSupplier request)
        {
            return Run([this, request = std::move(request)]
                       { return SetControl(request()); });
        }

        /**
         * \brief Returns a command that applies the specified control request to this swervedrivetrain->
         *
         * \param request Function returning the request to apply
         * \returns Command to run
         */
        template <typename RequestSupplier>
            requires std::negation_v<std::is_lvalue_reference<std::invoke_result_t<RequestSupplier>>> &&
                     requires(RequestSupplier req, TunerSwerveDrivetrain &drive) { drive.SetControl(req()); }
        frc2::CommandPtr ApplyRequest(RequestSupplier request)
        {
            return Run([this, request = std::move(request)]
                       { return SetControl(request()); });
        }

        void Periodic() override;

        /**
         * \brief Adds a vision measurement to the Kalman Filter. This will correct the
         * odometry pose estimate while still accounting for measurement noise.
         *
         * \param visionRobotPose The pose of the robot as measured by the vision camera.
         * \param timestamp The timestamp of the vision measurement in seconds.
         */
        void AddVisionMeasurement(frc::Pose2d visionRobotPose, units::second_t timestamp)
        {
            TunerSwerveDrivetrain::AddVisionMeasurement(std::move(visionRobotPose), utils::FPGAToCurrentTime(timestamp));
        }

        /**
         * \brief Adds a vision measurement to the Kalman Filter. This will correct the
         * odometry pose estimate while still accounting for measurement noise.
         *
         * Note that the vision measurement standard deviations passed into this method
         * will continue to apply to future measurements until a subsequent call to
         * #SetVisionMeasurementStdDevs or this method.
         *
         * \param visionRobotPose The pose of the robot as measured by the vision camera.
         * \param timestamp The timestamp of the vision measurement in seconds.
         * \param visionMeasurementStdDevs Standard deviations of the vision pose measurement.
         */
        void AddVisionMeasurement(
            frc::Pose2d visionRobotPose,
            units::second_t timestamp,
            std::array<double, 3> visionMeasurementStdDevs)
        {
            TunerSwerveDrivetrain::AddVisionMeasurement(std::move(visionRobotPose), utils::FPGAToCurrentTime(timestamp), visionMeasurementStdDevs);
        }

        // Hand Coded
        frc::Pose2d GetPose() { return this->GetState().Pose; };

        bool IsSamePose();

        void ResetSamePose()
        {
            m_debounceTimer.Reset();
            m_prevPose = GetPose();
        }
        // Hand Coded
    private:
        void StartSimThread();
        frc::Timer m_debounceTimer;
        const units::time::second_t m_samePoseTime = 0.5_s;
        const units::length::inch_t m_distanceThreshold{0.25};
        frc::Pose2d m_prevPose;
    };

}
