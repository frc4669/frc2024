// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"
#include "frc4669.h"
#include <ctre/phoenix6/controls/Follower.hpp>
#include <units/voltage.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/time.h>
#include <frc/DriverStation.h>

Drivetrain::Drivetrain() {
  frc4669::ConfigureMotor(m_leftMainMotor, false);
  frc4669::ConfigureMotor(m_leftSecondaryMotor, false);
  m_leftSecondaryMotor.SetControl(m_leftFollower);

  frc4669::ConfigureMotor(m_rightMainMotor, true);
  frc4669::ConfigureMotor(m_rightSecondaryMotor, true);
  m_rightSecondaryMotor.SetControl(m_rightFollower);
  ConfigureRamsete();
  
  m_drive.SetSafetyEnabled(false);

  m_timer.Reset();
  m_timer.Start();
}

frc2::CommandPtr Drivetrain::SetSpeedMutiplier(double mutiplier) {
  return RunOnce(
    [this, mutiplier] {this->m_driveSpeedMutiplier = mutiplier;}
  );
}

frc2::CommandPtr Drivetrain::SetTurnMutiplier(double mutiplier) {
  return RunOnce(
    [this, mutiplier] {this->m_turnSpeedMutiplier = mutiplier;}
  );
}

// This method will be called once per scheduler run
void Drivetrain::Periodic() {
  // m_drive.Feed();
  m_odometry.Update(frc::Rotation2d(GetYaw()), GetLeftDistance(), GetRightDistance());
  m_rotation = frc::Rotation2d(GetYaw());

  frc::SmartDashboard::PutNumber("Yaw", GetYaw().value());
  frc::SmartDashboard::PutNumber("Pitch", GetPitch().value());

  frc::SmartDashboard::PutNumber("muti", m_driveSpeedMutiplier);

  frc::Pose2d pose = Drivetrain::OdometryPose();
}

void Drivetrain::CurvatureDrive(double forward, double rotation){
  m_drive.CurvatureDrive(forward, rotation, m_motorTurnInPlace);
}

// tank drive but voltes
void Drivetrain::TankDriveVolts(units::volt_t left, units::volt_t right) {
  m_leftMainMotor.SetVoltage(left);
  m_leftSecondaryMotor.SetVoltage(left);
  m_rightMainMotor.SetVoltage(right);
  m_rightSecondaryMotor.SetVoltage(right);
}

// default command to drive based on logi controller inputs
frc2::CommandPtr Drivetrain::DefaultDriveCommand(std::function<double()> speed, std::function<double()> rotation) {
    return Run([this, speed = std::move(speed), rotation = std::move(rotation)]{
      CurvatureDrive(speed()  * m_driveSpeedMutiplier, rotation() * OperatorConstants::kTurningSpeedMutiplier); 
    });
}

// drive the robot using a chassis speed with PID and feedforward
void Drivetrain::DriveChassisSpeed(frc::ChassisSpeeds speeds){
  units::second_t currentTime = m_timer.Get();
  units::second_t delta = currentTime - m_prevTime;
  m_prevTime = currentTime;

  
  frc::DifferentialDriveWheelSpeeds wheelSpeeds =  m_kinematics.ToWheelSpeeds(speeds);
  frc::DifferentialDriveWheelSpeeds currentSpeed = GetWheelSpeeds();
  units::volt_t leftFF = m_feedforward.Calculate(wheelSpeeds.left, (wheelSpeeds.left - currentSpeed.left) / delta);
  units::volt_t rightFF = m_feedforward.Calculate(wheelSpeeds.right, (wheelSpeeds.right - currentSpeed.right) / delta);

  units::volt_t leftOut = units::volt_t(m_leftController.Calculate(currentSpeed.left.value(), wheelSpeeds.left.value()));
  units::volt_t rightOut = units::volt_t(m_rightController.Calculate(currentSpeed.right.value(), wheelSpeeds.right.value()));

  TankDriveVolts(leftOut + leftFF, rightOut + rightFF);
}

// returns wheels speeds for both side of dt
frc::DifferentialDriveWheelSpeeds Drivetrain::GetWheelSpeeds() {
  units::meters_per_second_t leftVelocity = units::meters_per_second_t(
    m_leftMainMotor.GetRotorVelocity().GetValue() * DriveConstants::kMetersPerTick
  );

  units::meters_per_second_t rightVelocity = units::meters_per_second_t(
    -(m_rightMainMotor.GetRotorVelocity().GetValue() * DriveConstants::kMetersPerTick)
  );

  return { leftVelocity, rightVelocity };
}

units::meter_t Drivetrain::GetLeftDistance() {
  return units::meter_t(m_leftMainMotor.GetPosition().GetValue() * DriveConstants::kMetersPerTick);
}

units::meter_t Drivetrain::GetRightDistance() {
  return units::meter_t(-(m_rightMainMotor.GetPosition().GetValue() * DriveConstants::kMetersPerTick));
}

units::degree_t Drivetrain::GetYaw() {
  return units::degree_t(-m_IMU.GetYaw()) - m_yawOffset;
}

units::degree_t Drivetrain::GetPitch() {
  return units::degree_t(m_IMU.GetPitch());
}

frc::Pose2d Drivetrain::OdometryPose() {
  return m_odometry.GetPose();
}

void Drivetrain::ResetEncoders() {
  m_leftMainMotor.SetPosition(units::angle::turn_t(0));
  m_rightMainMotor.SetPosition(units::angle::turn_t(0));
}

void Drivetrain::ResetOdometry(frc::Pose2d pose) {
  ResetEncoders();
  m_IMU.ZeroYaw();
  m_yawOffset = m_rotation.Degrees();
  m_odometry.ResetPosition(m_rotation, 0_m, 0_m, pose);
}

frc2::CommandPtr Drivetrain::AutonomousCommand(){
    auto path = pathplanner::PathPlannerPath::fromPathFile("Example Path");

    return pathplanner::AutoBuilder::followPath(path);
}

void Drivetrain::ConfigureRamsete() {
  pathplanner::AutoBuilder::configureRamsete(
    [this] (){ return m_odometry.GetPose(); },
    [this] (frc::Pose2d pose) { ResetOdometry(pose); },
    [this] () { return m_kinematics.ToChassisSpeeds(Drivetrain::GetWheelSpeeds()); },
    [this](frc::ChassisSpeeds speeds){ DriveChassisSpeed(speeds); },
    pathplanner::ReplanningConfig(),
    []() {
        auto alliance = frc::DriverStation::GetAlliance();
        if (alliance) {
            return alliance.value() == frc::DriverStation::Alliance::kRed;
        }
        return false;
    },
    this
  );
}

