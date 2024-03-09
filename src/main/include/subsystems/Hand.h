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
  void GoToSetPoint();

  frc2::CommandPtr Place();
  frc2::CommandPtr Intake();

  frc2::CommandPtr TurnNote(double pos);
  frc2::CommandPtr StopHand();

  frc2::CommandPtr SetWristPos(double pos);
  frc2::CommandPtr SetElevPos(double pos);
  
  

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_topMotor { CAN::kHandTopMotor, rev::CANSparkMax::MotorType::kBrushless };
  rev::CANSparkMax m_bottomMotor { CAN::kHandBottomMotor, rev::CANSparkMax::MotorType::kBrushless };
  ctre::phoenix6::hardware::TalonFX m_wristMotor { CAN::kHandRotationMotor };
  ctre::phoenix6::hardware::TalonFX m_elevMotor { CAN::kHandElevatorMotor };

  double m_I_wri = WristConstants::kI; 
  double m_P_wri = WristConstants::kP;
  double m_D_wri = WristConstants::kD;
  ctre::phoenix6::controls::PositionDutyCycle m_wristMotMagic {0_tr};

  double m_P_elev = ElevatorConstants::kP;
  double m_I_elev = ElevatorConstants::kI; 
  double m_D_elev = ElevatorConstants::kD;
  ctre::phoenix6::controls::PositionDutyCycle m_elevMotMagic {0_tr};

  frc::PIDController m_unviPID {0, 0, 0};

  rev::SparkMaxRelativeEncoder m_topEncoder; 
  rev::SparkMaxRelativeEncoder m_bottomEncoder;

  double m_P_hand = 0.05; 
  double m_I_hand = 0;
  double m_D_hand = 0.1;

  double m_rampRate = HandConstants::rampRate;

  double targetTurn = 0;
};
