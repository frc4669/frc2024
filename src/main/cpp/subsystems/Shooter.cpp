// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <units/angular_velocity.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <frc2/command/WaitUntilCommand.h>

Shooter::Shooter() {
    frc4669::ConfigureMotor( m_topMotor, false );
    frc4669::ConfigureMotor( m_bottomMotor, false );
    frc::SmartDashboard::PutBoolean("Runig", false);
    frc::SmartDashboard::PutNumber("power", power);

    // top motor config 
    {
    ctre::phoenix6::configs::TalonFXConfiguration topConfig {}; 
    m_velTopMotMag.Slot = 0; 
    topConfig.Slot0.kP = ShooterConstants::kTopP; 
    topConfig.Slot0.kI = 0.0; 
    topConfig.Slot0.kD = ShooterConstants::kTopD;
    topConfig.Slot0.kV = ShooterConstants::kTopV;
    topConfig.Slot0.kS = ShooterConstants::kTopS;
    topConfig.Slot0.kA = ShooterConstants::kTopA; 
    m_topMotor.GetConfigurator().Apply(topConfig);  
    }; 
    // bottom motor config 
    {
    ctre::phoenix6::configs::TalonFXConfiguration bottomConfig {}; 
    m_velBotMotMag.Slot = 0; 
    bottomConfig.Slot0.kI = 0.0; 
    bottomConfig.Slot0.kP = ShooterConstants::kBottomP; 
    bottomConfig.Slot0.kD = ShooterConstants::kBottomD;
    bottomConfig.Slot0.kV = ShooterConstants::kBottomV;
    bottomConfig.Slot0.kS = ShooterConstants::kBottomS;
    bottomConfig.Slot0.kA = ShooterConstants::kBottomA; 
    m_bottomMotor.GetConfigurator().Apply(bottomConfig); 
    }; 

    // m_bottomMotor.SetControl(m_groupFollwer); 

    frc::SmartDashboard::PutNumber("P_t", P_t);
    frc::SmartDashboard::PutNumber("FFV_t", 0);
    frc::SmartDashboard::PutNumber("D_t", D_t);

    frc::SmartDashboard::PutNumber("P_b", P_b);
    frc::SmartDashboard::PutNumber("FFV_b", 0);
    frc::SmartDashboard::PutNumber("D_b", D_b);
    
    frc::SmartDashboard::PutBoolean("UPDATE", true);
    frc::SmartDashboard::PutNumber("ianisbad", 0); 
    frc::SmartDashboard::PutNumber("targetShotVel", 0);

}

// This method will be called once per scheduler run
void Shooter::Periodic() {
  bool update = frc::SmartDashboard::GetBoolean("UPDATE", false);
  double newpower = frc::SmartDashboard::GetNumber("power", 0);
  double newPt = frc::SmartDashboard::GetNumber("P_t",P_t);  
  double newFFVt = frc::SmartDashboard::GetNumber("FFV_t",0);
  double newDt = frc::SmartDashboard::GetNumber("D_t",D_t);
  double newPb = frc::SmartDashboard::GetNumber("P_b",P_b);
  double newFFVb = frc::SmartDashboard::GetNumber("FFV_b",0);
  double newDb = frc::SmartDashboard::GetNumber("D_b",D_b);
  double targetVel = frc::SmartDashboard::GetNumber("targetShotVel", 0); 
  if (update) {
    if(power != newpower){
    power = newpower;
    }
    // top motor config 
    {
    ctre::phoenix6::configs::TalonFXConfiguration topConfig {}; 
    m_velTopMotMag.Slot = 0; 
    topConfig.Slot0.kP = newPt; 
    // topConfig.Slot0.kV = newFFVt;  
    topConfig.Slot0.kD = newDt;
    // topConfig.Slot0.k
    m_topMotor.GetConfigurator().Apply(topConfig);  
    }; 
    // bottom motor config 
    {
    ctre::phoenix6::configs::TalonFXConfiguration bottomConfig {}; 
    m_velBotMotMag.Slot = 0; 
    bottomConfig.Slot0.kP = newPb; 
    // bottomConfig.Slot0.kV = newFFVb;
    bottomConfig.Slot0.kD = newDb; 
    m_bottomMotor.GetConfigurator().Apply(bottomConfig); 
    }; 
  }
  
  
}

frc2::CommandPtr Shooter::ShootVel(units::turns_per_second_t top, units::turns_per_second_t bottom) {
  return RunOnce(
    [this, top, bottom] {
      // auto velocity = units::turns_per_second_t(this->power);
      this->m_topMotor.SetControl(this->m_velTopMotMag.WithVelocity(top)); 
      this->m_bottomMotor.SetControl(this->m_velBotMotMag.WithVelocity(bottom)); 
    }
  );
}

frc2::CommandPtr Shooter::Shoot (double output){
    return RunOnce(
      // [this, output] { 
      [this, output] {
        m_topMotor.Set(-(output));
        m_bottomMotor.Set(-output);
        units::angular_velocity::turns_per_second_t motorVelocity = m_topMotor.GetRotorVelocity().GetValue();
        frc::SmartDashboard::PutNumber("rotor vel", motorVelocity.value());
        // // m_bottomMotor.Set(output);

        // if(abs(motorVelocity.value()) >= 100){
        //   frc::SmartDashboard::PutBoolean("Runig", true);
          
        // }
      }
    );
}

frc2::CommandPtr Shooter::YeetStuckNote (double speed){
  return Run(
    [this, speed] {
      m_topMotor.Set(-(speed));
      m_bottomMotor.Set(speed);
    }
  ).WithTimeout(0.5_s)
  .AndThen(StopMotors());
}

frc2::CommandPtr Shooter::StopMotors (){
    return RunOnce(
      [this] {
        m_topMotor.Set(0.0);
        m_bottomMotor.Set(0.0);
      }
    );
}

frc2::CommandPtr Shooter::SysIdQuasistatic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Quasistatic(direction);
}

frc2::CommandPtr Shooter::SysIdDynamic(frc2::sysid::Direction direction) {
  return m_sysIdRoutine.Dynamic(direction);
}