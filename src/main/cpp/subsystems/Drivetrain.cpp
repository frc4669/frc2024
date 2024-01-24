// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"
#include "frc4669.h"
#include <ctre/phoenix6/controls/Follower.hpp>
#include <units/voltage.h>

Drivetrain::Drivetrain() {
    frc4669::ConfigureMotor(leftMainMotor, true);
    frc4669::ConfigureMotor(leftSecondaryMotor, true);
    leftSecondaryMotor.SetControl(leftFollower);

    frc4669::ConfigureMotor(rightMainMotor, false);
    frc4669::ConfigureMotor(rightSecondaryMotor, false);
    rightSecondaryMotor.SetControl(rightFollower);

}

// This method will be called once per scheduler run

void Drivetrain::Periodic() {}

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