// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"
#include "frc4669.h"
#include <ctre/phoenix6/controls/Follower.hpp>
#include <units/voltage.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/time.h>

Drivetrain::Drivetrain() {
    frc4669::ConfigureMotor(leftMainMotor, true);
    frc4669::ConfigureMotor(leftSecondaryMotor, true);
    leftSecondaryMotor.SetControl(leftFollower);

    frc4669::ConfigureMotor(rightMainMotor, false);
    frc4669::ConfigureMotor(rightSecondaryMotor, false);
    rightSecondaryMotor.SetControl(rightFollower);
}

// This method will be called once per scheduler run

void Drivetrain::Periodic() {
    m_odometry.Update(frc::Rotation2d(GetYaw()), GetLeftDistance(), GetRightDistance());

    frc::Pose2d robotPose = m_odometry.GetPose();

    m_field->SetRobotPose(robotPose);

    frc::SmartDashboard::PutNumber("Yaw", GetYaw().value());
    frc::SmartDashboard::PutNumber("Pitch", GetPitch().value());
}

void Drivetrain::CurvatureDrive(double forward, double rotation){
    drive.CurvatureDrive(forward, rotation, motorTurnInPlace);
}

void Drivetrain::TankDriveVolts(units::volt_t left, units::volt_t right) {
    leftMainMotor.SetVoltage(left);
    leftSecondaryMotor.SetVoltage(left);
    rightMainMotor.SetVoltage(right);
    rightSecondaryMotor.SetVoltage(right);
}

frc2::CommandPtr Drivetrain::DefaultDriveCommand(std::function<double()> speed, std::function<double()> rotation)
{
    return Run([this, speed = std::move(speed), rotation = std::move(rotation)]{
        drive.CurvatureDrive(speed(), rotation(), true);
    });
}

frc::DifferentialDriveWheelSpeeds Drivetrain::GetWheelSpeeds() {
  units::meters_per_second_t leftVelocity = units::meters_per_second_t(
    leftMainMotor.GetRotorVelocity().GetValue() * 10 * DriveConstants::kMetersPerTick
  );

  units::meters_per_second_t rightVelocity = units::meters_per_second_t(
    -(rightMainMotor.GetRotorVelocity().GetValue() * 10 * DriveConstants::kMetersPerTick)
  );

  return { leftVelocity, rightVelocity };
}

units::meter_t Drivetrain::GetLeftDistance() {
  return units::meter_t(leftMainMotor.GetPosition().GetValue() * DriveConstants::kMetersPerTick);
}

units::meter_t Drivetrain::GetRightDistance() {
  return units::meter_t(-(rightMainMotor.GetPosition().GetValue() * DriveConstants::kMetersPerTick));
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
  leftMainMotor.SetPosition(units::angle::turn_t(0));
  rightMainMotor.SetPosition(units::angle::turn_t(0));
}

void Drivetrain::ResetOdometry(frc::Pose2d pose, frc::Rotation2d rotation) {
  ResetEncoders();
  m_IMU.ZeroYaw();
  m_yawOffset = rotation.Degrees();
  m_odometry.ResetPosition(rotation, 0_m, 0_m, pose);
}