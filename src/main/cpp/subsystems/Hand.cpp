// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Hand.h"
#include "frc4669.h"
#include <frc/smartdashboard/SmartDashboard.h>

Hand::Hand() {
    frc4669::ConfigureRevMotor(topMotor, false);
    frc4669::ConfigureRevMotor(bottomMotor, true);
    bottomMotor.Follow(topMotor);
    bottomMotor.SetClosedLoopRampRate(rampRate);
    topMotor.SetClosedLoopRampRate(rampRate);
}

// This method will be called once per scheduler run
void Hand::Periodic() {}

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

}

// turns hand with PID
frc2::CommandPtr Hand::GoToDownPos () {
    return RunOnce([this] { this->EnsureInvert(false); }).AndThen(
        Run(
            [this] {
                this->topMotor.Set(0.1); 
            }
        ).Until([this] { return !this->revLimit.Get(); })
        .AndThen(StopHand())
    ); 
}

// top and bottom needs to be going the same direction
// current test config INVERT: bottom needs to be negated 
frc2::CommandPtr Hand::GoToUpPos () {
    return RunOnce([this] { this->EnsureInvert(false); }).AndThen(
        Run(
            [this] {
                this->topMotor.Set(-0.1); 
            }
        ).Until([this] { return !this->fwdLimit.Get(); })
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
