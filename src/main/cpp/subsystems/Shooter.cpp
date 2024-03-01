// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <units/angular_velocity.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

Shooter::Shooter() {
    frc4669::ConfigureMotor( mainMotor, false );
    frc4669::ConfigureMotor( minorMotor, false );
    frc::SmartDashboard::PutBoolean("Runig", false);
    frc::SmartDashboard::PutNumber("power", power);
    minorMotor.SetControl(groupFollwer); 
}

// This method will be called once per scheduler run
void Shooter::Periodic() {
  double newpower = frc::SmartDashboard::GetNumber("power", 0);
  if(power != newpower){
    power = newpower;
  }
    
}


frc2::CommandPtr Shooter::Shoot (double output){
    return Run(
      // [this, output] { 
      [this] {
        double output = this->power;
        mainMotor.Set(-output);
        units::angular_velocity::turns_per_second_t motorVelocity = mainMotor.GetRotorVelocity().GetValue();
        frc::SmartDashboard::PutNumber("rotor vel", motorVelocity.value());
        // // minorMotor.Set(output);

        // if(abs(motorVelocity.value()) >= 100){
        //   frc::SmartDashboard::PutBoolean("Runig", true);
          
        // }
      }
    );
}
frc2::CommandPtr Shooter::StopMotors (){
    return Run(
      [this] {
        mainMotor.Set(0.0);
        // minorMotor.Set(0.0);
      }
    );
}


// create a motor -> setup 
