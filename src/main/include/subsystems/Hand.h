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

class Hand : public frc2::SubsystemBase {
 public:
  Hand();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void GoToSetPoint(double setpoint);

  frc2::CommandPtr Place ();
  frc2::CommandPtr TurnNote ();
  frc2::CommandPtr HandTurn ();
  frc2::CommandPtr StopHand ();
  
  

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax topMotor { CAN::kHandTopMotor, rev::CANSparkMax::MotorType::kBrushless };
  rev::CANSparkMax bottomMotor { CAN::kHandBottomMotor, rev::CANSparkMax::MotorType::kBrushless };

  rev::SparkMaxPIDController topPID;
  rev::SparkMaxPIDController bottomPID;
  frc::PIDController unviPID {0.1, 0, 7.0};

  rev::SparkMaxRelativeEncoder topEncoder; 
  rev::SparkMaxRelativeEncoder bottomEncoder;

  double P = 0.1; 
  double D = 7.0;
  double targetRot = 0; 
  double targetTurn = 0;

  double minOutCur = -1; 
  double maxOutCur = 1;
  double rampRate = 1;
};
