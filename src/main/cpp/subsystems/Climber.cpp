// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"
#include "frc4669.h"
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/voltage.h>

Climber::Climber() {
    frc4669::ConfigureMotor(ClimbMotor1, false);
    frc4669::ConfigureMotor(ClimbMotor2, false);
    ClimbMotor2.SetControl(groupFollower);
    
    // motor config
    ctre::phoenix6::configs::HardwareLimitSwitchConfigs fwdLimitConfig {};
    // have the limit switch auto reset the encoder for zeroing
    fwdLimitConfig.ForwardLimitAutosetPositionEnable = true; 
    fwdLimitConfig.ForwardLimitAutosetPositionValue = 0; 
    fwdLimitConfig.ForwardLimitEnable = true; // hard limit 

    // motion magic
    rotMotMagic.Slot = 0;    
    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfigs{};

    auto& slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = this->P;
    slot0Configs.kI = this->I;
    slot0Configs.kD = this->D;

    auto& motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 120; // turns per second --> 3 turns on the actual shaft 
    motionMagicConfigs.MotionMagicAcceleration = 80; // turns per second ^2 --> 2 tps^2

    ClimbMotor1.GetConfigurator().Apply(talonFXConfigs, 50_ms); 

    frc::SmartDashboard::PutBoolean( "Climb Complete", false );
    frc::SmartDashboard::PutNumber("P_climber", P); 
    frc::SmartDashboard::PutNumber("I_climber", I);
    frc::SmartDashboard::PutNumber("D_climber", D);
}

// This method will be called once per scheduler run
void Climber::Periodic() {

    frc::SmartDashboard::PutBoolean( "Climb Complete", true);

    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfigs{};
    bool configChanged = false;

    double newP = frc::SmartDashboard::GetNumber("P_climber", 0);
    double newD = frc::SmartDashboard::GetNumber("I_climber", 0);
    double newI = frc::SmartDashboard::GetNumber("D_climber", 0);

    if (newP != P) {
        this->P = newP; 
        talonFXConfigs.Slot0.kP = newP; 
        configChanged = true;
    }
    if (newD != D) {
        this->D = newD; 
        talonFXConfigs.Slot0.kD = newD; 
        configChanged = true;
    }
    if (newI != I) {
        this->I = newI; 
        talonFXConfigs.Slot0.kI = newI; 
        configChanged = true;
    }
    if (configChanged) ClimbMotor1.GetConfigurator().Apply(talonFXConfigs, 50_ms); 
}

frc2::CommandPtr Climber::ZeroClimber() {
    return Run(
        [this] {
            this->ClimbMotor1.Set(0.1); // set to a resonable homing speed
        }
    );
    // ).Until( [this] { return ClimbMotor1.GetForwardLimit().GetValue() == ctre::phoenix6::signals::ForwardLimitValue::ClosedToGround; });
    // encoder reset will happen automatically with the current motor configuration
}

frc2::CommandPtr Climber::StartClimb(){
    return Run(
        [this] {
            this->ClimbMotor1.SetControl(this->rotMotMagic.WithPosition(-300_tr));
        }
    )
    // velocity check for when we hits the hard stop (should just kill the velocity)
    // .Until([this] { return std::abs(this->ClimbMotor1.GetVelocity().GetValueAsDouble()) < 0.1 && std::abs(this->ClimbMotor1.GetPosition().GetValue().value() - std::abs(120)) < 40; }) // units in turns
    // .AndThen(Run( // turn back down to pull the robot up
    //     [this] {
    //         this->rotMotMagic.WithPosition(0_tr);
    //     })
    // ) 
    // // we'll hit the limit switch and that will kill the velocity again
    // .Until([this] { return std::abs(this->ClimbMotor1.GetVelocity().GetValueAsDouble()) < 0.1; })
    .AndThen([this] { this->climbComplete = true; });
};

// enter PID Hold at position
frc2::CommandPtr Climber::StopClimb(){
    return Run(
        [this] {
            this->ClimbMotor1.SetControl(this->rotMotMagic.WithPosition(this->ClimbMotor1.GetPosition().GetValue()));
        }
    );
};

bool Climber::ClimbComplete (){
    return this->climbComplete;
};