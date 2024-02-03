// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Hand.h"
#include "frc4669.h"
#include <frc/smartdashboard/SmartDashboard.h>

Hand::Hand(): 
    topPID(topMotor.GetPIDController()), 
    bottomPID(bottomMotor.GetPIDController()),
    topEncoder(topMotor.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42)),
    bottomEncoder(bottomMotor.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42))
{
    frc4669::ConfigureRevMotor(topMotor, false);
    frc4669::ConfigureRevMotor(bottomMotor, true);

    bottomPID.SetP(0);
    bottomPID.SetI(0);
    bottomPID.SetD(0);
    bottomPID.SetOutputRange(minOutCur, maxOutCur); 
    bottomMotor.SetClosedLoopRampRate(rampRate);
    topPID.SetP(0); 
    topPID.SetI(0); 
    topPID.SetD(0); 
    topPID.SetOutputRange(minOutCur, maxOutCur);
    topMotor.SetClosedLoopRampRate(rampRate);


    frc::SmartDashboard::PutNumber("P", P); 
    frc::SmartDashboard::PutNumber("D", D); 
    frc::SmartDashboard::PutNumber("TargetRot", targetRot);
    frc::SmartDashboard::PutNumber("TargetTurn", targetTurn);
    frc::SmartDashboard::PutBoolean("UPDATE", false);
    frc::SmartDashboard::PutNumber("MinOutCur", minOutCur); 
    frc::SmartDashboard::PutNumber("MaxOutCur", maxOutCur);
    frc::SmartDashboard::PutNumber("Ramp Rate", rampRate);
}

// This method will be called once per scheduler run
void Hand::Periodic() {
    if (frc::SmartDashboard::GetBoolean("UPDATE", false)) {
        double newP = frc::SmartDashboard::GetNumber("P", 0);
        double newD = frc::SmartDashboard::GetNumber("D", 0); 
        double newTargetRot = frc::SmartDashboard::GetNumber("TargetRot", 0); 
        double newTargetTurn = frc::SmartDashboard::GetNumber("TargetTurn", 0); 
        double newMinCur = frc::SmartDashboard::GetNumber("MinOutCur", 0); 
        double newMaxCur = frc::SmartDashboard::GetNumber("MaxOutCur", 0); 
        double newRampRate = frc::SmartDashboard::PutNumber("RampRate", 0);
        if (newRampRate != rampRate) {
            this->rampRate = newRampRate;
            topMotor.SetClosedLoopRampRate(rampRate);
            bottomMotor.SetClosedLoopRampRate(rampRate);
        }
        if (newMinCur != minOutCur) {
            this->minOutCur = newMinCur; 
            bottomPID.SetOutputRange(minOutCur, maxOutCur); 
            topPID.SetOutputRange(minOutCur, maxOutCur);
        }
        // if (newP != P) {
        //     this->P = newP; 
        //     topPID.SetP(this->P); 
        //     bottomPID.SetP(this->P);
        // }
        // if (newD != D) {
        //     this->D = newD; 
        //     topPID.SetD(this->D); 
        //     bottomPID.SetD(this->D);
        // }
        if (newTargetRot != targetRot) {
            this->targetRot = newTargetRot;
        }
        if (newTargetTurn != targetTurn) {
            this->targetTurn = newTargetTurn;
        }
    }

}

frc2::CommandPtr Hand::Place () {
    return RunOnce(
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
    return RunOnce(
        [this] {
            topPID.SetReference(targetRot, rev::ControlType::kPosition);
            bottomPID.SetReference(targetRot, rev::ControlType::kPosition);
        }
    );
}

// top and bottom needs to be going the same direction
// current test config INVERT: bottom needs to be negated 
frc2::CommandPtr Hand::TurnNote () {
    return RunOnce(
        [this] {
            topPID.SetReference(targetRot + targetTurn, rev::ControlType::kPosition);
            bottomPID.SetReference(targetRot - targetTurn, rev::ControlType::kPosition);
        }
    );
}
frc2::CommandPtr Hand::StopHand (){
    return RunOnce(
        [this] {
            topMotor.Set(0.0);
            bottomMotor.Set(0.0);
        }
    );
}

void Hand::GoToSetPoint(double setpoint) {
    units::volt_t output = unviPID.Calculate((topEncoder.GetPosition() + bottomEncoder.GetPosition())/2, setpoint);
    topMotor.SetVoltage(output);
}