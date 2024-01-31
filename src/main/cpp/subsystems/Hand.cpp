// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Hand.h"
#include "frc4669.h"

Hand::Hand() {
    frc4669::ConfigureRevMotor(topMotor, false);
    frc4669::ConfigureRevMotor(bottomMotor, true);
    bottomMotor.Follow(topMotor, true);
}

// This method will be called once per scheduler run
void Hand::Periodic() {}

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
frc2::CommandPtr Hand::HandTurn () {
    //Position detection probably
    return Run(
        [this] {

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