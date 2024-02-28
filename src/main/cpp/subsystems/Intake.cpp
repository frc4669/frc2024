// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"
#include "frc4669.h"
#include <frc/smartdashboard/SmartDashboard.h>

Intake::Intake() {
    frc4669::ConfigureMotor(intakeMotor, false);
    frc4669::ConfigureMotor(feederMotor, false); 
    frc::SmartDashboard::PutBoolean( "Intake Complete", false);
};

// This method will be called once per scheduler run
void Intake::Periodic() {}

frc2::CommandPtr Intake::StartIntake(double output){
    return Run(
        [this, output] {
            intakeMotor.Set(output);
            if (isIntakeCompleted) {
                StopIntake();
            }
        }
    ).Until([this] {if(isIntakeCompleted) {return true;}})
    .AndThen(StopIntake());
};

frc2::CommandPtr Intake::StopIntake(){
    return Run(
        [this] {
            intakeMotor.Set(0.0);
        }
    );
}

frc2::CommandPtr Intake::StopFeeder(){
    return Run(
        [this] {
            feederMotor.Set(0.0);
        }
    );
}


frc2::CommandPtr Intake::RunFeeder() {
    return Run(
        [this] {
            this->feederMotor.Set(0.20); 
        }
    ).WithTimeout(0.25_s)
    .AndThen(StopFeeder()); 
}

bool Intake::IntakeComplete(){
    frc::SmartDashboard::PutBoolean("Intake Complete", true);
};