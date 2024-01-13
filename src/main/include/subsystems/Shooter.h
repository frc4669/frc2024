// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include "Constants.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/smartdashboard/SmartDashboard.h>
class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void ConfigureMotor(ctre::phoenix6::hardware::TalonFX &motor, bool isInverted);
  frc2::CommandPtr RunMotor (double output);
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  ctre::phoenix6::hardware::TalonFX mainMotor { CAN::kMainMotor };
  ctre::phoenix6::hardware::TalonFX minorMotor { CAN::kMainMotor };

  ctre::phoenix6::controls::Follower groupFollwer { CAN::kMainMotor, false};
 
  frc::MotorControllerGroup shooterMotors { mainMotor, minorMotor };

}; 
