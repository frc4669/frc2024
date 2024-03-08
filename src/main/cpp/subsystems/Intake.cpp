// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"
#include "frc4669.h"
#include <frc/smartdashboard/SmartDashboard.h>

Intake::Intake() {
    frc4669::ConfigureMotor(m_intakeMotor, true);
    frc4669::ConfigureMotor(m_feederMotor, true);
    m_timer.Reset();
    m_timer.Start();
    this->m_lastTimestamp = m_timer.Get();
};

// This method will be called once per scheduler run
void Intake::Periodic() {
    // units::second_t curTime = m_timer.Get(); 
    // // auto matically update feeder velocity tracking to be able to recover after unexpected behavior
    // if ((curTime - m_lastTimestamp) > 0.5_s) {
    //     m_lastTimestamp = curTime; 
    //     m_lastLowestVelocity = units::math::abs(GetFeederVelocity());
    // }
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

units::turns_per_second_t Intake::GetFeederVelocity() {
    return m_feederMotor.GetVelocity().GetValue(); 
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

// a command to wait until the feeder have detected some kind of slow down
frc2::CommandPtr Intake::WaitUntilFeederCollision() {
    this->m_lastLowestVelocity = units::math::abs(GetFeederVelocity()); 
    return Run([this] {
        units::turns_per_second_t curVelocity = units::math::abs(GetFeederVelocity()); 
        if (curVelocity < this->m_lastLowestVelocity) {
            this->m_lastLowestVelocity = curVelocity; 
        }
    }).Until([this] {
        return this->m_lastLowestVelocity <= FeederConstants::kLowestNominalVelocity;
    });
}
