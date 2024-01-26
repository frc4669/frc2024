// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Intake.h"
#include "frc4669.h"
#include <frc2/command/CommandPtr.h>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>

Intake::Intake() {
    frc4669::ConfigureMotor( intakeMotor, false);
};

// This method will be called once per scheduler run
void Intake::Periodic() {}

frc2::CommandPtr Intake::StartIntake(double output){
    return Run(
        [this, output] {
            intakeMotor.Set(output);
        }
    );
};

frc2::CommandPtr Intake::StopIntake(){
    return Run(
        [this] {
            intakeMotor.Set(0);
        }
    );
}
