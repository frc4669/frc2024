// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <Constants.h>
#include <units/voltage.h>
#include <units/angle.h>
#include <frc/SPI.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc2/command/CommandPtr.h>
#include <ctre/phoenix6/controls/Follower.hpp>

//setup 4 motors for drive 
// 

class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain();
    
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void CurvatureDrive(double forward, double rotation);
  void TankDriveVolts(units::volt_t left, units::volt_t right);

  frc2::CommandPtr DefaultDriveCommand(std::function<double()> speed, std::function<double()> rotation);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix6::hardware::TalonFX leftMainMotor { CAN::kLeftMainMotor };
  ctre::phoenix6::hardware::TalonFX leftSecondaryMotor { CAN::kLeftSecondaryMotor };
  ctre::phoenix6::hardware::TalonFX rightMainMotor { CAN::kRightMainMotor };
  ctre::phoenix6::hardware::TalonFX rightSecondaryMotor { CAN::kRightSecondaryMotor };

  ctre::phoenix6::controls::Follower leftFollower { CAN::kLeftMainMotor, false };
  ctre::phoenix6::controls::Follower rightFollower { CAN::kRightMainMotor, false};

  frc::DifferentialDrive drive { leftMainMotor, rightMainMotor };

  bool motorTurnInPlace = true;
};
