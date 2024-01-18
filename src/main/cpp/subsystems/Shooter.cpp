// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Shooter.h"
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <units/angular_velocity.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>

Shooter::Shooter() {
    ConfigureMotor( mainMotor, true );
    ConfigureMotor( minorMotor, true );
    frc::SmartDashboard::PutBoolean("Runig", false);
//   minorMotor.SetControl(groupFollwer); 
}; 

// This method will be called once per scheduler run
void Shooter::Periodic() {
}


frc2::CommandPtr Shooter::Shoot (double output){
    return Run(
      [this, output] { 
        mainMotor.Set(output);
        units::angular_velocity::turns_per_second_t motorVelocity = mainMotor.GetRotorVelocity().GetValue();
        frc::SmartDashboard::PutNumber("rotor vel", motorVelocity.value());
        if(abs(motorVelocity.value()) >= 100){
          frc::SmartDashboard::PutBoolean("Runig", true);
          
          minorMotor.Set(output);
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


void Shooter::ConfigureMotor(ctre::phoenix6::hardware::TalonFX &motor, bool isInverted) {
  ctre::phoenix6::configs::TalonFXConfiguration talonFXConfigs{};
  ctre::phoenix6::configs::CurrentLimitsConfigs currentLimitsConfigs{};
  ctre::phoenix6::configs::MotorOutputConfigs motorOutputConfigs{};

  // Motor current configuration (exceeding these limits generally damages the motor)
  currentLimitsConfigs.SupplyCurrentLimitEnable = true;
  currentLimitsConfigs.SupplyCurrentLimit = 25;
  currentLimitsConfigs.SupplyCurrentThreshold = 25;
  currentLimitsConfigs.SupplyTimeThreshold = 0.5;

  // Neutral mode configuration (sets motor to passively reduce motion without other input)
  motorOutputConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

  // Reverses default motor direction (needed if motor is orientated in an undesirable position)
  motorOutputConfigs.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

  //! safety is enabled

  
  
  motor.GetConfigurator().Apply(ctre::phoenix6::configs::TalonFXConfiguration{});

  // Configures time taken to move motor from neutral to full power
//   motor.ConfigOpenloopRamp(0.1);
//   motor.ConfigClosedloopRamp(0);
}



// create a motor -> setup 
