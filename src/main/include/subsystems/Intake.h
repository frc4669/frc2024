// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/Timer.h>
#include "Constants.h"

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  frc2::CommandPtr IntakeNote();

  frc2::CommandPtr StartIntake(double speed);
  frc2::CommandPtr StopIntake();
  frc2::CommandPtr StartFeeder(double speed);
  frc2::CommandPtr StopFeeder();
  units::turns_per_second_t GetFeederVelocity(); 
  frc2::CommandPtr WaitUntilFeederCollision(); 

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  ctre::phoenix6::hardware::TalonFX m_intakeMotor { CAN::kIntakeMotor };
  ctre::phoenix6::hardware::TalonFX m_feederMotor { CAN::kFeederMotor };
  
  frc::Timer m_timer; 
  units::second_t m_lastTimestamp;
  units::turns_per_second_t m_lastLowestVelocity;
  
  bool isIntakeCompleted = false;
};
