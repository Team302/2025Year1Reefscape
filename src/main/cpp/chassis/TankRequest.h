#pragma once

#include <memory>
#include <vector>
#include <algorithm>

#include "units/angle.h"
#include "units/angular_velocity.h"
#include "units/length.h"
#include "units/math.h"
#include "units/time.h"
#include "units/velocity.h"
#include "units/voltage.h"

#include "frc/geometry/Rotation2d.h"

using namespace frc;

namespace drive
{
    namespace tank
    {
        namespace requests
        {

            enum class DriveRequestType
            {
                OpenLoopVoltage,
                Velocity,
            };

            // Chassis speeds (forward + yaw rate)
            struct ChassisSpeeds
            {
                units::meters_per_second_t vx{0_mps};
                units::radians_per_second_t omega{0_rad_per_s};
            };

            // Field-centric forward perspective
            enum class ForwardPerspectiveValue
            {
                RED_ALLIANCE = 0,
                BLUE_ALLIANCE = 1,
            };

            // Control parameters
            struct ControlParameters
            {
                units::meters_per_second_t kMaxSpeed{0_mps};
                units::meter_t kTrackWidth{0_m};
                Rotation2d kCurrentHeading{};
            };

            class TankDriveSide
            {
            public:
                virtual ~TankDriveSide() = default;
                virtual void ApplyDrive(units::meters_per_second_t speed, DriveRequestType type) = 0;
                virtual void ApplyVoltage(units::volt_t volts) = 0;
            };

            // Base request
            class TankDriveRequest
            {
            public:
                using ControlParams = ControlParameters;
                virtual ~TankDriveRequest() = default;
                virtual void Apply(ControlParams const &parameters,
                                   std::vector<std::unique_ptr<TankDriveSide>> const &sidesToApply) = 0;
            };

            // Idle
            class Idle : public TankDriveRequest
            {
            public:
                void Apply(ControlParams const &,
                           std::vector<std::unique_ptr<TankDriveSide>> const &) override {}
            };

            // Brake
            class TankDriveBrake : public TankDriveRequest
            {
            public:
                DriveRequestType DriveType = DriveRequestType::OpenLoopVoltage;

                void Apply(ControlParams const &,
                           std::vector<std::unique_ptr<TankDriveSide>> const &sidesToApply) override
                {
                    for (auto const &side : sidesToApply)
                        if (side)
                            side->ApplyDrive(0_mps, DriveType);
                }

                TankDriveBrake &WithDriveRequestType(DriveRequestType t) &
                {
                    DriveType = t;
                    return *this;
                }
                TankDriveBrake &&WithDriveRequestType(DriveRequestType t) &&
                {
                    DriveType = t;
                    return std::move(*this);
                }
            };

            // Field-centric
            class FieldCentric : public TankDriveRequest
            {
            };

            // Robot-centric
            class RobotCentric : public TankDriveRequest
            {
            };

            // Apply robot-centric speeds
            class ApplyRobotSpeeds : public TankDriveRequest
            {
            };

            // Apply field-centric speeds
            class ApplyFieldSpeeds : public TankDriveRequest
            {
            };

            // Field-centric facing angle
            class FieldCentricFacingAngle : public TankDriveRequest
            {
            };

            // Robot-centric facing angle
            class RobotCentricFacingAngle : public TankDriveRequest
            {
            };

        } // namespace requests
    } // namespace tank
} // namespace drive