// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "Constants.h"
#include <frc/controller/PIDController.h>
#include <frc/Encoder.h>
#include <units/angle.h>

class Wall : public frc2::SubsystemBase {
 public:
  Wall();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  frc2::CommandPtr SetWallPos(units::turn_t targetPos);
  frc2::CommandPtr StopWall();
  frc2::CommandPtr StopMotors();
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  ctre::phoenix6::hardware::TalonFX m_wallMotor { CAN::kWallMotor };
  
  double m_P = WallConstants::kP;
  double m_I = WallConstants::kI; 
  double m_D = WallConstants::kD;
  ctre::phoenix6::controls::PositionDutyCycle m_rotMotMagic {0_tr};
};
