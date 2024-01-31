// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <units/angular_velocity.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

Shooter::Shooter() {
    frc4669::ConfigureMotor( mainMotor, true );
    frc4669::ConfigureMotor( minorMotor, true );
    frc::SmartDashboard::PutBoolean("Runig", false);
//   minorMotor.SetControl(groupFollwer); 
}

// This method will be called once per scheduler run
void Shooter::Periodic() {
}


frc2::CommandPtr Shooter::Shoot (double output){
    return Run(
      [this, output] { 
        mainMotor.Set(output);
        units::angular_velocity::turns_per_second_t motorVelocity = mainMotor.GetRotorVelocity().GetValue();
        frc::SmartDashboard::PutNumber("rotor vel", motorVelocity.value());
        minorMotor.Set(output);

        if(abs(motorVelocity.value()) >= 100){
          frc::SmartDashboard::PutBoolean("Runig", true);
          
        }
      }
    );
}
frc2::CommandPtr Shooter::StopMotors (){
    return Run(
      [this] {
        mainMotor.Set(0.0);
        minorMotor.Set(0.0);
      }
    );
}


// create a motor -> setup 
