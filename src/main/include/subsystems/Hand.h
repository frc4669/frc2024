// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include <rev/CANPIDController.h>
#include <frc/controller/PIDController.h>
#include <frc/DigitalInput.h>

class Hand : public frc2::SubsystemBase {
 public:
  Hand();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void EnsureInvert(bool inverted);

  frc2::CommandPtr Place ();
  frc2::CommandPtr GoToUpPos ();
  frc2::CommandPtr GoToDownPos ();
  frc2::CommandPtr StopHand ();
  
  

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax topMotor { CAN::kHandTopMotor, rev::CANSparkMax::MotorType::kBrushless };
  rev::CANSparkMax bottomMotor { CAN::kHandBottomMotor, rev::CANSparkMax::MotorType::kBrushless };

  frc::DigitalInput fwdLimit { DIO::kHandFwdLimit }; 
  frc::DigitalInput revLimit { DIO::kHandRevLimit };

  double rampRate = 1;
};
