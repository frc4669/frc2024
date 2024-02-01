// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Hand.h"
#include "frc4669.h"
#include <frc/smartdashboard/SmartDashboard.h>

Hand::Hand(): topPID(topMotor.GetPIDController()), bottomPID(bottomMotor.GetPIDController()) {
    frc4669::ConfigureRevMotor(topMotor, false);
    frc4669::ConfigureRevMotor(bottomMotor, true);
    // bottomMotor.Follow(topMotor, true);

    bottomPID.SetP(P);
    bottomPID.SetI(0);
    bottomPID.SetD(D);
    topPID.SetP(P); 
    topPID.SetI(0); 
    topPID.SetD(D); 

    frc::SmartDashboard::PutNumber("P", P); 
    frc::SmartDashboard::PutNumber("D", D); 
    frc::SmartDashboard::PutNumber("TargetRot", targetRot);
    frc::SmartDashboard::PutBoolean("UPDATE", false);
}

// This method will be called once per scheduler run
void Hand::Periodic() {
    if (frc::SmartDashboard::GetBoolean("UPDATE", false)) {
        double newP = frc::SmartDashboard::GetNumber("P", 0);
        double newD = frc::SmartDashboard::GetNumber("D", 0); 
        double newTargetRot = frc::SmartDashboard::GetNumber("TargetRot", 0); 
        if (newP != P) {
            this->P = newP; 
            topPID.SetP(this->P); 
        }
        if (newD != D) {
            this->D = newD; 
            topPID.SetD(this->D); 
        }
        if (newTargetRot != targetRot) {
            this->targetRot = newTargetRot;
        }
    }

}

frc2::CommandPtr Hand::HandIn () {
    return Run(
        [this] {
            topMotor.Set(-0.05);
            bottomMotor.Set(-0.05);
            
        }
    // ).OnlyWhile(
    //     [this] {
    //         if (0/*placeholder*/ < topMotor.GetOutputCurrent()) {
    //             topMotor.Set(0.0);
    //             bottomMotor.Set(0.0);
    //             return false;
    //         }
    //         return true;
    //     }
    );
}

// turns hand with PID
frc2::CommandPtr Hand::HandTurn () {
    return Run(
        [this] {
            topPID.SetReference(targetRot, rev::ControlType::kPosition);
            bottomPID.SetReference(targetRot, rev::ControlType::kPosition);
        }
    );
}
frc2::CommandPtr Hand::HandOut () {
    //Don't know if detection for the note leaving the hand is possible so timer?
    return Run(
        [this] {
            topMotor.Set(0.2); 
            bottomMotor.Set(0.2);
        }
    );
}
frc2::CommandPtr Hand::StopHand (){
    return Run(
        [this] {
            topMotor.Set(0.0);
            bottomMotor.Set(0.0);
        }
    );
}