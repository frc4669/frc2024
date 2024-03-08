// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"
#include "frc4669.h"
#include <frc/smartdashboard/SmartDashboard.h>

Intake::Intake() {
    frc4669::ConfigureMotor(m_intakeMotor, true);
    frc4669::ConfigureMotor(m_feederMotor, true); 
};

// This method will be called once per scheduler run
void Intake::Periodic() {
}

// Intake 
frc2::CommandPtr Intake::IntakeNote() {
    return StartIntake(0.5);
}

frc2::CommandPtr Intake::StartIntake(double speed){
    return RunOnce(
        [this, speed] {
            m_intakeMotor.Set(speed);
            // if (isIntakeCompleted) {
            //     StopIntake();
            // }
        }
    );
    // ).Until([this] {if(isIntakeCompleted) {return true;}})
    // .AndThen(StopIntake());
};

frc2::CommandPtr Intake::StopIntake(){
    return RunOnce(
        [this] {
            m_intakeMotor.Set(0.0);
        }
    );
}

frc2::CommandPtr Intake::StopFeeder(){
    return RunOnce(
        [this] {
            m_feederMotor.Set(0.0);
        }
    );
}


frc2::CommandPtr Intake::StartFeeder(double speed) {
    return RunOnce(
        [this, speed] {
            this->m_feederMotor.Set(speed); 
        }
    // ).WithTimeout(0.25_s)
    // .AndThen(StopFeeder()); 
    );
}

bool Intake::IntakeComplete(){
    frc::SmartDashboard::PutBoolean("Intake Complete", true);
};