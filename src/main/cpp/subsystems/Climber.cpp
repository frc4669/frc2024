// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"
#include "frc4669.h"
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>

Climber::Climber() {
    frc4669::ConfigureMotor(ClimbMotor1, false);
    frc4669::ConfigureMotor(ClimbMotor2, false);
    frc::SmartDashboard::PutBoolean( "Climb Complete", false );
}

// This method will be called once per scheduler run
void Climber::Periodic() {}

frc2::CommandPtr Climber::StartClimb(){
    return Run(
        [this] {
            
        }
    );
};

frc2::CommandPtr Climber::StopClimb(){
    return Run(
        [this] {

        }
    );
};

void ClimbComplete (){
    frc::SmartDashboard::PutBoolean( "Climb Complete", true);
};