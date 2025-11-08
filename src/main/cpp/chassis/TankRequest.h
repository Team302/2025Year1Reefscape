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

namespace drive {
namespace tank {
namespace requests {

// Drive request type for tank sides
enum class DriveRequestType {
    OpenLoopVoltage,
    Velocity,
};

// Chassis speeds (forward + yaw rate)
struct ChassisSpeeds {
    units::meters_per_second_t vx{0_mps};
    units::radians_per_second_t omega{0_rad_per_s};
};

// Simple PID controller
class PIDController {
public:
    double kP{0.0}, kI{0.0}, kD{0.0};
    double integral{0.0};
    double prevError{0.0};

    PIDController() = default;
    PIDController(double p, double i, double d) : kP(p), kI(i), kD(d) {}

    double Calculate(double measurement, double setpoint, double dt = 0.02) {
        double error = setpoint - measurement;
        integral += error * dt;
        double derivative = (error - prevError) / dt;
        prevError = error;
        return kP * error + kI * integral + kD * derivative;
    }
};

// Field-centric forward perspective
enum class ForwardPerspectiveValue {
    OperatorPerspective = 0,
    BlueAlliance = 1,
};

// Control parameters
struct ControlParameters {
    units::meters_per_second_t kMaxSpeed{0_mps};
    units::meter_t kTrackWidth{0_m};
    Rotation2d kCurrentHeading{};
};

// Tank drive side interface
class TankDriveSide {
public:
    virtual ~TankDriveSide() = default;
    virtual void ApplyDrive(units::meters_per_second_t speed, DriveRequestType type) = 0;
    virtual void ApplyVoltage(units::volt_t volts) = 0;
};

// Base request
class TankDriveRequest {
public:
    using ControlParams = ControlParameters;
    virtual ~TankDriveRequest() = default;
    virtual void Apply(ControlParams const& parameters,
                       std::vector<std::unique_ptr<TankDriveSide>> const& sidesToApply) = 0;
};

// Idle
class Idle : public TankDriveRequest {
public:
    void Apply(ControlParams const&,
               std::vector<std::unique_ptr<TankDriveSide>> const&) override {}
};

// Brake
class TankDriveBrake : public TankDriveRequest {
public:
    DriveRequestType DriveType = DriveRequestType::OpenLoopVoltage;

    void Apply(ControlParams const&,
               std::vector<std::unique_ptr<TankDriveSide>> const& sidesToApply) override {
        for (auto const& side : sidesToApply)
            if (side) side->ApplyDrive(0_mps, DriveType);
    }

    TankDriveBrake& WithDriveRequestType(DriveRequestType t) & { DriveType = t; return *this; }
    TankDriveBrake&& WithDriveRequestType(DriveRequestType t) && { DriveType = t; return std::move(*this); }
};

// Field-centric
class FieldCentric : public TankDriveRequest {
public:
    units::meters_per_second_t VelocityForward{0_mps};
    units::radians_per_second_t RotationalRate{0_rad_per_s};
    units::meters_per_second_t Deadband{0_mps};
    units::radians_per_second_t RotationalDeadband{0_rad_per_s};

    DriveRequestType DriveType = DriveRequestType::OpenLoopVoltage;
    bool DesaturateWheelSpeeds = true;
    ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue::OperatorPerspective;

    void Apply(ControlParams const& parameters,
               std::vector<std::unique_ptr<TankDriveSide>> const& sidesToApply) override {
        auto v = VelocityForward;
        auto w = RotationalRate;

        if (units::math::abs(v) < Deadband) v = 0_mps;
        if (units::math::abs(w) < RotationalDeadband) w = 0_rad_per_s;

        auto turnLever = parameters.kTrackWidth / 2.0;
        auto vLeft  = v - turnLever * w;
        auto vRight = v + turnLever * w;

        if (DesaturateWheelSpeeds && parameters.kMaxSpeed > 0_mps) {
            auto maxMag = units::math::max(units::math::abs(vLeft), units::math::abs(vRight));
            if (maxMag > parameters.kMaxSpeed) {
                auto scale = parameters.kMaxSpeed / maxMag;
                vLeft *= scale;
                vRight *= scale;
            }
        }

        if (sidesToApply.size() >= 2) {
            if (sidesToApply[0]) sidesToApply[0]->ApplyDrive(vLeft, DriveType);
            if (sidesToApply[1]) sidesToApply[1]->ApplyDrive(vRight, DriveType);
        }
    }

    FieldCentric& WithVelocityX(units::meters_per_second_t v) & { VelocityForward = v; return *this; }
    FieldCentric&& WithVelocityX(units::meters_per_second_t v) && { VelocityForward = v; return std::move(*this); }
    FieldCentric& WithRotationalRate(units::radians_per_second_t w) & { RotationalRate = w; return *this; }
    FieldCentric&& WithRotationalRate(units::radians_per_second_t w) && { RotationalRate = w; return std::move(*this); }
    FieldCentric& WithDeadband(units::meters_per_second_t db) & { Deadband = db; return *this; }
    FieldCentric&& WithDeadband(units::meters_per_second_t db) && { Deadband = db; return std::move(*this); }
    FieldCentric& WithRotationalDeadband(units::radians_per_second_t db) & { RotationalDeadband = db; return *this; }
    FieldCentric&& WithRotationalDeadband(units::radians_per_second_t db) && { RotationalDeadband = db; return std::move(*this); }
    FieldCentric& WithDriveRequestType(DriveRequestType t) & { DriveType = t; return *this; }
    FieldCentric&& WithDriveRequestType(DriveRequestType t) && { DriveType = t; return std::move(*this); }
    FieldCentric& WithDesaturateWheelSpeeds(bool d) & { DesaturateWheelSpeeds = d; return *this; }
    FieldCentric&& WithDesaturateWheelSpeeds(bool d) && { DesaturateWheelSpeeds = d; return std::move(*this); }
    FieldCentric& WithForwardPerspective(ForwardPerspectiveValue fp) & { ForwardPerspective = fp; return *this; }
    FieldCentric&& WithForwardPerspective(ForwardPerspectiveValue fp) && { ForwardPerspective = fp; return std::move(*this); }
};

// Robot-centric
class RobotCentric : public TankDriveRequest {
public:
    units::meters_per_second_t VelocityForward{0_mps};
    units::radians_per_second_t RotationalRate{0_rad_per_s};
    units::meters_per_second_t Deadband{0_mps};
    units::radians_per_second_t RotationalDeadband{0_rad_per_s};

    DriveRequestType DriveType = DriveRequestType::OpenLoopVoltage;
    bool DesaturateWheelSpeeds = true;

    void Apply(ControlParams const& parameters,
               std::vector<std::unique_ptr<TankDriveSide>> const& sidesToApply) override {
        auto v = VelocityForward;
        auto w = RotationalRate;

        if (units::math::abs(v) < Deadband) v = 0_mps;
        if (units::math::abs(w) < RotationalDeadband) w = 0_rad_per_s;

        auto turnLever = parameters.kTrackWidth / 2.0;
        auto vLeft  = v - turnLever * w;
        auto vRight = v + turnLever * w;

        if (DesaturateWheelSpeeds && parameters.kMaxSpeed > 0_mps) {
            auto maxMag = units::math::max(units::math::abs(vLeft), units::math::abs(vRight));
            if (maxMag > parameters.kMaxSpeed) {
                auto scale = parameters.kMaxSpeed / maxMag;
                vLeft *= scale;
                vRight *= scale;
            }
        }

        if (sidesToApply.size() >= 2) {
            if (sidesToApply[0]) sidesToApply[0]->ApplyDrive(vLeft, DriveType);
            if (sidesToApply[1]) sidesToApply[1]->ApplyDrive(vRight, DriveType);
        }
    }

    RobotCentric& WithVelocityX(units::meters_per_second_t v) & { VelocityForward = v; return *this; }
    RobotCentric&& WithVelocityX(units::meters_per_second_t v) && { VelocityForward = v; return std::move(*this); }
    RobotCentric& WithRotationalRate(units::radians_per_second_t w) & { RotationalRate = w; return *this; }
    RobotCentric&& WithRotationalRate(units::radians_per_second_t w) && { RotationalRate = w; return std::move(*this); }
    RobotCentric& WithDeadband(units::meters_per_second_t db) & { Deadband = db; return *this; }
    RobotCentric&& WithDeadband(units::meters_per_second_t db) && { Deadband = db; return std::move(*this); }
    RobotCentric& WithRotationalDeadband(units::radians_per_second_t db) & { RotationalDeadband = db; return *this; }
    RobotCentric&& WithRotationalDeadband(units::radians_per_second_t db) && { RotationalDeadband = db; return std::move(*this); }
    RobotCentric& WithDriveRequestType(DriveRequestType t) & { DriveType = t; return *this; }
    RobotCentric&& WithDriveRequestType(DriveRequestType t) && { DriveType = t; return std::move(*this); }
    RobotCentric& WithDesaturateWheelSpeeds(bool d) & { DesaturateWheelSpeeds = d; return *this; }
    RobotCentric&& WithDesaturateWheelSpeeds(bool d) && { DesaturateWheelSpeeds = d; return std::move(*this); }
};

// Apply robot-centric speeds
class ApplyRobotSpeeds : public TankDriveRequest {
public:
    ChassisSpeeds Speeds{};
    DriveRequestType DriveType = DriveRequestType::OpenLoopVoltage;
    bool DesaturateWheelSpeeds = true;

    void Apply(ControlParams const& parameters,
               std::vector<std::unique_ptr<TankDriveSide>> const& sidesToApply) override {
        RobotCentric rc;
        rc.VelocityForward = Speeds.vx;
        rc.RotationalRate  = Speeds.omega;
        rc.DesaturateWheelSpeeds = DesaturateWheelSpeeds;
        rc.DriveType = DriveType;
        rc.Apply(parameters, sidesToApply);
    }

    ApplyRobotSpeeds& WithSpeeds(ChassisSpeeds s) & { Speeds = s; return *this; }
    ApplyRobotSpeeds&& WithSpeeds(ChassisSpeeds s) && { Speeds = s; return std::move(*this); }
    ApplyRobotSpeeds& WithDriveRequestType(DriveRequestType t) & { DriveType = t; return *this; }
    ApplyRobotSpeeds&& WithDriveRequestType(DriveRequestType t) && { DriveType = t; return std::move(*this); }
    ApplyRobotSpeeds& WithDesaturateWheelSpeeds(bool d) & { DesaturateWheelSpeeds = d; return *this; }
    ApplyRobotSpeeds&& WithDesaturateWheelSpeeds(bool d) && { DesaturateWheelSpeeds = d; return std::move(*this); }
};

// Apply field-centric speeds
class ApplyFieldSpeeds : public TankDriveRequest {
public:
    ChassisSpeeds Speeds{};
    DriveRequestType DriveType = DriveRequestType::OpenLoopVoltage;
    bool DesaturateWheelSpeeds = true;
    ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue::BlueAlliance;

    void Apply(ControlParams const& parameters,
               std::vector<std::unique_ptr<TankDriveSide>> const& sidesToApply) override {
        FieldCentric fc;
        fc.VelocityForward = Speeds.vx;
        fc.RotationalRate  = Speeds.omega;
        fc.DriveType = DriveType;
        fc.DesaturateWheelSpeeds = DesaturateWheelSpeeds;
        fc.ForwardPerspective = ForwardPerspective;
        fc.Apply(parameters, sidesToApply);
    }

    ApplyFieldSpeeds& WithSpeeds(ChassisSpeeds s) & { Speeds = s; return *this; }
    ApplyFieldSpeeds&& WithSpeeds(ChassisSpeeds s) && { Speeds = s; return std::move(*this); }
    ApplyFieldSpeeds& WithDriveRequestType(DriveRequestType t) & { DriveType = t; return *this; }
    ApplyFieldSpeeds&& WithDriveRequestType(DriveRequestType t) && { DriveType = t; return std::move(*this); }
    ApplyFieldSpeeds& WithDesaturateWheelSpeeds(bool d) & { DesaturateWheelSpeeds = d; return *this; }
    ApplyFieldSpeeds&& WithDesaturateWheelSpeeds(bool d) && { DesaturateWheelSpeeds = d; return std::move(*this); }
    ApplyFieldSpeeds& WithForwardPerspective(ForwardPerspectiveValue fp) & { ForwardPerspective = fp; return *this; }
    ApplyFieldSpeeds&& WithForwardPerspective(ForwardPerspectiveValue fp) && { ForwardPerspective = fp; return std::move(*this); }
};

// Field-centric facing angle
class FieldCentricFacingAngle : public TankDriveRequest {
public:
    units::meters_per_second_t VelocityForward{0_mps};
    Rotation2d TargetDirection{};
    units::radians_per_second_t TargetRateFeedforward{0_rad_per_s};
    units::meters_per_second_t Deadband{0_mps};
    units::radians_per_second_t RotationalDeadband{0_rad_per_s};
    units::radians_per_second_t MaxAbsRotationalRate{0_rad_per_s};

    DriveRequestType DriveType = DriveRequestType::OpenLoopVoltage;
    bool DesaturateWheelSpeeds = true;
    ForwardPerspectiveValue ForwardPerspective = ForwardPerspectiveValue::OperatorPerspective;
    PIDController HeadingController{0, 0, 0};

    void Apply(ControlParams const& parameters,
               std::vector<std::unique_ptr<TankDriveSide>> const& sidesToApply) override {
        double omegaCmd = HeadingController.Calculate(parameters.kCurrentHeading.Radians().value(),
                                                      TargetDirection.Radians().value());
        omegaCmd += TargetRateFeedforward.value();
        if (MaxAbsRotationalRate > 0_rad_per_s) {
            omegaCmd = std::clamp(omegaCmd,
                                  (-MaxAbsRotationalRate).value(),
                                  MaxAbsRotationalRate.value());
        }

        FieldCentric fc;
        fc.VelocityForward = VelocityForward;
        fc.RotationalRate = units::radians_per_second_t{omegaCmd};
        fc.Deadband = Deadband;
        fc.RotationalDeadband = RotationalDeadband;
        fc.DriveType = DriveType;
        fc.DesaturateWheelSpeeds = DesaturateWheelSpeeds;
        fc.ForwardPerspective = ForwardPerspective;
        fc.Apply(parameters, sidesToApply);
    }

    FieldCentricFacingAngle& WithVelocityX(units::meters_per_second_t v) & { VelocityForward = v; return *this; }
    FieldCentricFacingAngle&& WithVelocityX(units::meters_per_second_t v) && { VelocityForward = v; return std::move(*this); }
    FieldCentricFacingAngle& WithTargetDirection(Rotation2d t) & { TargetDirection = t; return *this; }
    FieldCentricFacingAngle&& WithTargetDirection(Rotation2d t) && { TargetDirection = t; return std::move(*this); }
    FieldCentricFacingAngle& WithTargetRateFeedforward(units::radians_per_second_t ff) & { TargetRateFeedforward = ff; return *this; }
    FieldCentricFacingAngle&& WithTargetRateFeedforward(units::radians_per_second_t ff) && { TargetRateFeedforward = ff; return std::move(*this); }
    FieldCentricFacingAngle& WithDeadband(units::meters_per_second_t db) & { Deadband = db; return *this; }
    FieldCentricFacingAngle&& WithDeadband(units::meters_per_second_t db) && { Deadband = db; return std::move(*this); }
    FieldCentricFacingAngle& WithRotationalDeadband(units::radians_per_second_t db) & { RotationalDeadband = db; return *this; }
    FieldCentricFacingAngle&& WithRotationalDeadband(units::radians_per_second_t db) && { RotationalDeadband = db; return std::move(*this); }
    FieldCentricFacingAngle& WithMaxAbsRotationalRate(units::radians_per_second_t m) & { MaxAbsRotationalRate = m; return *this; }
    FieldCentricFacingAngle&& WithMaxAbsRotationalRate(units::radians_per_second_t m) && { MaxAbsRotationalRate = m; return std::move(*this); }
    FieldCentricFacingAngle& WithDriveRequestType(DriveRequestType t) & { DriveType = t; return *this; }
    FieldCentricFacingAngle&& WithDriveRequestType(DriveRequestType t) && { DriveType = t; return std::move(*this); }
    FieldCentricFacingAngle& WithDesaturateWheelSpeeds(bool d) & { DesaturateWheelSpeeds = d; return *this; }
    FieldCentricFacingAngle&& WithDesaturateWheelSpeeds(bool d) && { DesaturateWheelSpeeds = d; return std::move(*this); }
    FieldCentricFacingAngle& WithForwardPerspective(ForwardPerspectiveValue fp) & { ForwardPerspective = fp; return *this; }
    FieldCentricFacingAngle&& WithForwardPerspective(ForwardPerspectiveValue fp) && { ForwardPerspective = fp; return std::move(*this); }
    FieldCentricFacingAngle& WithHeadingController(PIDController c) & { HeadingController = c; return *this; }
    FieldCentricFacingAngle&& WithHeadingController(PIDController c) && { HeadingController = c; return std::move(*this); }
};

// Robot-centric facing angle
class RobotCentricFacingAngle : public TankDriveRequest {
public:
    units::meters_per_second_t VelocityForward{0_mps};
    Rotation2d TargetDirection{};
    units::radians_per_second_t TargetRateFeedforward{0_rad_per_s};
    units::meters_per_second_t Deadband{0_mps};
    units::radians_per_second_t RotationalDeadband{0_rad_per_s};
    units::radians_per_second_t MaxAbsRotationalRate{0_rad_per_s};

    DriveRequestType DriveType = DriveRequestType::OpenLoopVoltage;
    bool DesaturateWheelSpeeds = true;
    PIDController HeadingController{0, 0, 0};

    void Apply(ControlParams const& parameters,
               std::vector<std::unique_ptr<TankDriveSide>> const& sidesToApply) override {
        double omegaCmd = HeadingController.Calculate(parameters.kCurrentHeading.Radians().value(),
                                                      TargetDirection.Radians().value());
        omegaCmd += TargetRateFeedforward.value();
        if (MaxAbsRotationalRate > 0_rad_per_s) {
            omegaCmd = std::clamp(omegaCmd,
                                  (-MaxAbsRotationalRate).value(),
                                  MaxAbsRotationalRate.value());
        }

        RobotCentric rc;
        rc.VelocityForward = VelocityForward;
        rc.RotationalRate = units::radians_per_second_t{omegaCmd};
        rc.Deadband = Deadband;
        rc.RotationalDeadband = RotationalDeadband;
        rc.DriveType = DriveType;
        rc.DesaturateWheelSpeeds = DesaturateWheelSpeeds;
        rc.Apply(parameters, sidesToApply);
    }

    RobotCentricFacingAngle& WithVelocityX(units::meters_per_second_t v) & { VelocityForward = v; return *this; }
    RobotCentricFacingAngle&& WithVelocityX(units::meters_per_second_t v) && { VelocityForward = v; return std::move(*this); }
    RobotCentricFacingAngle& WithTargetDirection(Rotation2d t) & { TargetDirection = t; return *this; }
    RobotCentricFacingAngle&& WithTargetDirection(Rotation2d t) && { TargetDirection = t; return std::move(*this); }
    RobotCentricFacingAngle& WithTargetRateFeedforward(units::radians_per_second_t ff) & { TargetRateFeedforward = ff; return *this; }
    RobotCentricFacingAngle&& WithTargetRateFeedforward(units::radians_per_second_t ff) && { TargetRateFeedforward = ff; return std::move(*this); }
    RobotCentricFacingAngle& WithDeadband(units::meters_per_second_t db) & { Deadband = db; return *this; }
    RobotCentricFacingAngle&& WithDeadband(units::meters_per_second_t db) && { Deadband = db; return std::move(*this); }
    RobotCentricFacingAngle& WithRotationalDeadband(units::radians_per_second_t db) & { RotationalDeadband = db; return *this; }
    RobotCentricFacingAngle&& WithRotationalDeadband(units::radians_per_second_t db) && { RotationalDeadband = db; return std::move(*this); }
    RobotCentricFacingAngle& WithMaxAbsRotationalRate(units::radians_per_second_t m) & { MaxAbsRotationalRate = m; return *this; }
    RobotCentricFacingAngle&& WithMaxAbsRotationalRate(units::radians_per_second_t m) && { MaxAbsRotationalRate = m; return std::move(*this); }
    RobotCentricFacingAngle& WithDriveRequestType(DriveRequestType t) & { DriveType = t; return *this; }
    RobotCentricFacingAngle&& WithDriveRequestType(DriveRequestType t) && { DriveType = t; return std::move(*this); }
    RobotCentricFacingAngle& WithDesaturateWheelSpeeds(bool d) & { DesaturateWheelSpeeds = d; return *this; }
    RobotCentricFacingAngle&& WithDesaturateWheelSpeeds(bool d) && { DesaturateWheelSpeeds = d; return std::move(*this); }
    RobotCentricFacingAngle& WithHeadingController(PIDController c) & { HeadingController = c; return *this; }
    RobotCentricFacingAngle&& WithHeadingController(PIDController c) && { HeadingController = c; return std::move(*this); }
};

} // namespace requests
} // namespace tank
} // namespace drive