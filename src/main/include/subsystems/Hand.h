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
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/MotionMagicDutyCycle.hpp>

class Hand : public frc2::SubsystemBase {
 public:
  Hand();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void EnsureInvert(bool inverted);
  void GoToSetPoint(double setpoint);

  frc2::CommandPtr Place ();
  frc2::CommandPtr TurnNote ();
  frc2::CommandPtr StopHand ();

  frc2::CommandPtr HandTurn ();

  frc2::CommandPtr RaiseHand();
  
  frc2::CommandPtr Intake();
  

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax topMotor { CAN::kHandTopMotor, rev::CANSparkMax::MotorType::kBrushless };
  rev::CANSparkMax bottomMotor { CAN::kHandBottomMotor, rev::CANSparkMax::MotorType::kBrushless };
  ctre::phoenix6::hardware::TalonFX rotMotor { CAN::kHandRotationMotor };
  ctre::phoenix6::hardware::TalonFX elevMotor { CAN::kHandElevatorMotor };

  double P_rot = 0.03;
  double I_rot = 0; 
  double D_rot = 0.0;
  ctre::phoenix6::controls::PositionDutyCycle rotMotMagic {0_tr};

  double P_elev = 0.03;
  double I_elev = 0; 
  double D_elev = 0.0;
  ctre::phoenix6::controls::PositionDutyCycle elevMotMagic {0_tr};

  rev::SparkMaxPIDController topPID;
  rev::SparkMaxPIDController bottomPID;
  frc::PIDController unviPID {0, 0, 0};

  rev::SparkMaxRelativeEncoder topEncoder; 
  rev::SparkMaxRelativeEncoder bottomEncoder;

  double P = 0.01; 
  double I = 0;
  double D = 0.0;
  double targetRot = 0; 
  double targetTurn = 0;
  double targetElev = 0;

  double minOutCur = -1; 
  double maxOutCur = 1;
  double rampRate = 1;
  double outCurLimit = 5;
};
