// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "Constants.h"

class Climber : public frc2::SubsystemBase {
 public:
  Climber();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  frc2::CommandPtr ZeroClimber();
  frc2::CommandPtr StartClimb();
  frc2::CommandPtr StopClimb();
  bool ClimbComplete ();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  ctre::phoenix6::hardware::TalonFX ClimbMotor1 { CAN::kClimbMotor1 };
  ctre::phoenix6::hardware::TalonFX ClimbMotor2 { CAN::kClimbMotor2 };
  ctre::phoenix6::controls::Follower groupFollower { CAN::kClimbMotor1, false }; 

  double P = 0.01;
  double I = 0; 
  double D = 0.0;
  ctre::phoenix6::controls::PositionDutyCycle rotMotMagic {0_tr};

  bool climbComplete = false;

};
