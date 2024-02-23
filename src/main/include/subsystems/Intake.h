// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include "Constants.h"

class Intake : public frc2::SubsystemBase {
 public:
  Intake();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  frc2::CommandPtr StartIntake (double output);
  frc2::CommandPtr StopIntake ();
  bool IntakeComplete ();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  ctre::phoenix6::hardware::TalonFX intakeMotor { CAN::kIntakeMotor };
};
