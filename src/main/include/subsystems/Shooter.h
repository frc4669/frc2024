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
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/RobotController.h>

#include "frc4669.h"
class Shooter : public frc2::SubsystemBase {
 public:
  Shooter();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  frc2::CommandPtr StopMotors ();
  frc2::CommandPtr Shoot (double output);
  frc2::CommandPtr ShootVel(units::turns_per_second_t velocity);
  frc2::CommandPtr YeetStuckNote(double speed); 

  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction);
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ctre::phoenix6::controls::VelocityDutyCycle m_velTopMotMag {units::turns_per_second_t(0)};
  ctre::phoenix6::controls::VelocityDutyCycle m_velBotMotMag {units::turns_per_second_t(0)};

  ctre::phoenix6::hardware::TalonFX m_topMotor { CAN::kShooterTop };
  ctre::phoenix6::hardware::TalonFX m_bottomMotor { CAN::kShooterBottom };

  ctre::phoenix6::controls::Follower m_groupFollwer { CAN::kShooterTop, false};

  frc2::sysid::SysIdRoutine m_sysIdRoutine {
    frc2::sysid::Config(1_V / 1_s, 8_V, 8_s, std::nullopt),
    frc2::sysid::Mechanism(
      [this] (units::volt_t driveVoltage) {
        m_bottomMotor.SetVoltage(driveVoltage);
      },
      [this] (frc::sysid::SysIdRoutineLog* log) {
        log->Motor("m_bottomMotor")
          .voltage(m_bottomMotor.Get() * frc::RobotController::GetBatteryVoltage())
          .position(m_bottomMotor.GetPosition().GetValue())
          .velocity(m_bottomMotor.GetVelocity().GetValue());
      }, this
    )
  };

  double P_t = ShooterConstants::kTopP;  
  double I_t = 0;
  double D_t = ShooterConstants::kTopD;
  double P_b = ShooterConstants::kBottomP;
  double I_b = 0;
  double D_b = ShooterConstants::kBottomD;  

  double power = 0;//0.33;
}; 
