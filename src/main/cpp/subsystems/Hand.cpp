// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Hand.h"
#include "frc4669.h"
#include <frc/smartdashboard/SmartDashboard.h>

Hand::Hand(): 
    topEncoder(topMotor.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42)),
    bottomEncoder(bottomMotor.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42))
{
    frc4669::ConfigureRevMotor(topMotor, false);
    frc4669::ConfigureRevMotor(bottomMotor, true);

    bottomMotor.SetClosedLoopRampRate(rampRate);
    topMotor.SetClosedLoopRampRate(rampRate);
    bottomMotor.Follow(topMotor, true);

    frc::SmartDashboard::PutNumber("TargetRot", targetRot);
    frc::SmartDashboard::PutNumber("TargetTurn", targetTurn);
    frc::SmartDashboard::PutBoolean("UPDATE", false);
    frc::SmartDashboard::PutNumber("Ramp Rate", rampRate);
    frc::SmartDashboard::PutNumber("TurnTime", turnTimeMS);
}

// This method will be called once per scheduler run
void Hand::Periodic() {
    if (frc::SmartDashboard::GetBoolean("UPDATE", false)) {
        double newTargetRot = frc::SmartDashboard::GetNumber("TargetRot", 0); 
        double newTargetTurn = frc::SmartDashboard::GetNumber("TargetTurn", 0); 
        double newRampRate = frc::SmartDashboard::PutNumber("RampRate", 0);
        double newTurnTime = frc::SmartDashboard::PutNumber("TurnTime", 0);

        if (newRampRate != rampRate) {
            this->rampRate = newRampRate;
            topMotor.SetClosedLoopRampRate(rampRate);
            bottomMotor.SetClosedLoopRampRate(rampRate);
        }
        if (newTargetRot != targetRot) {
            this->targetRot = newTargetRot;
        }
        if (newTargetTurn != targetTurn) {
            this->targetTurn = newTargetTurn;
        }
        if (newTurnTime != turnTimeMS) {
            this->turnTimeMS = newTurnTime;
        }
    }

}

void Hand::EnsureInvert(bool inverted) {
    bottomMotor.Follow(topMotor, inverted);
}

frc2::CommandPtr Hand::Intake () {
    return RunOnce([this] { this->EnsureInvert(true); }).AndThen( 
        Run(
            [this] {
                topMotor.Set(0.05);
            }
        ).WithTimeout(500_ms)
        .AndThen(StopHand())
    );
}

frc2::CommandPtr Hand::Place () {
    return RunOnce([this] { this->EnsureInvert(true); }).AndThen( 
        Run(
            [this] {
                topMotor.Set(-0.05);
            }
        ).WithTimeout(500_ms)
        .AndThen(StopHand())
    );
    // ).OnlyWhile(
    //     [this] {
    //         if (0/*placeholder*/ < topMotor.GetOutputCurrent()) {
    //             topMotor.Set(0.0);
    //             bottomMotor.Set(0.0);
    //             return false;
    //         }
    //         return true;
    //     }
    // );
}

// top and bottom needs to be going the same direction
// current test config INVERT: bottom needs to be negated/ change normal follow mode
frc2::CommandPtr Hand::TurnNote () {
    pid.Calculate((topEncoder.GetPosition() + bottomEncoder.GetPosition()) / 2);
    return RunOnce([this] {this->EnsureInvert(false);}).AndThen(
        Run(
        [this] {
            topMotor.Set(this->targetRot);
        })
        .WithTimeout(units::millisecond_t(this->turnTimeMS))
        .AndThen(StopHand())
    );
}
frc2::CommandPtr Hand::StopHand (){
    return RunOnce(
        [this] {
            topMotor.Set(0.0);
        }
    );
}