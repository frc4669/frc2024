// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"
#include "frc4669.h"
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/voltage.h>

Climber::Climber() {
    frc4669::ConfigureMotor(ClimbMotor1, false);
    frc4669::ConfigureMotor(ClimbMotor2, false);
    frc::SmartDashboard::PutBoolean( "Climb Complete", false );
    frc::SmartDashboard::PutNumber("Climber kP", kP);
    frc::SmartDashboard::PutNumber("Climber kI", kI);
    frc::SmartDashboard::PutNumber("Climber kD", kD);
    bool climbComplete = false;
}

// This method will be called once per scheduler run
void Climber::Periodic() {
    double newkP = frc::SmartDashboard::GetNumber("Climber kP", kP);
    double newkI = frc::SmartDashboard::GetNumber("Climber kI", kI);
    double newkD = frc::SmartDashboard::GetNumber("Climber kD", kD);

    if(newkP != kP) {
        pid.SetP(newkP);
    }
    if(newkI != kI) {
        pid.SetI(newkI);
    }
    if(newkD != kD) {
        pid.SetD(newkD);
    }
}

units::angle::turn_t Climber::GetPosition1(){
    ctre::phoenix6::StatusSignal position = ClimbMotor1.GetPosition();
    return position.GetValue();
}

units::angle::turn_t Climber::GetPosition2(){
    ctre::phoenix6::StatusSignal position = ClimbMotor2.GetPosition();
    return position.GetValue();
}

frc2::CommandPtr Climber::StartClimb(double setpoint){
    return Run(
        [this, setpoint] {
            ClimbMotor1.SetVoltage(units::volt_t(pid.Calculate(GetPosition1().value(), setpoint)));
            ClimbMotor2.SetVoltage(units::volt_t(pid.Calculate(GetPosition2().value(), setpoint)));
        }
    );
};

frc2::CommandPtr Climber::StopClimb(){
    return Run(
        [this] {
            ClimbMotor1.Set(0.0);
            ClimbMotor2.Set(0.0);
        }
    );
};

void ClimbComplete (){
    frc::SmartDashboard::PutBoolean( "Climb Complete", true);
};