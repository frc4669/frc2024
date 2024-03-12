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

class Climber : public frc2::SubsystemBase {
 public:
  Climber();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  frc2::CommandPtr ZeroClimber();
  frc2::CommandPtr SetClimberPos(double targetPos);
  frc2::CommandPtr WaitUntillClimberHitHardStop();
  frc2::CommandPtr StopClimb();
  frc2::CommandPtr StopMotors();

  bool IsClimberAtBottom();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  ctre::phoenix6::hardware::TalonFX m_climbMotor1 { CAN::kClimbMotor1 };
  ctre::phoenix6::hardware::TalonFX m_climbMotor2 { CAN::kClimbMotor2 };
  ctre::phoenix6::controls::Follower m_groupFollower { CAN::kClimbMotor1, true }; 

  double m_P = ClimberConstants::kP;
  double m_I = ClimberConstants::kI; 
  double m_D = ClimberConstants::kD;
  ctre::phoenix6::controls::PositionDutyCycle m_rotMotMagic {0_tr};

  bool m_climbComplete = false;

};
