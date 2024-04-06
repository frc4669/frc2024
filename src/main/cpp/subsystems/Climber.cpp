// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Climber.h"
#include "frc4669.h"
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/WaitUntilCommand.h>
#include <units/voltage.h>
#include "Actions.h"

Climber::Climber() {
    frc4669::ConfigureMotor(m_climbMotor, false);
    
    // motor config
    ctre::phoenix6::configs::HardwareLimitSwitchConfigs fwdLimitConfig {};
    // have the limit switch auto reset the encoder for zeroing
    fwdLimitConfig.ForwardLimitAutosetPositionEnable = true; 
    fwdLimitConfig.ForwardLimitAutosetPositionValue = 0; 
    fwdLimitConfig.ForwardLimitEnable = true; // hard limit 

    // motion magic
    m_rotMotMagic.Slot = 0;    
    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfigs{};

    auto& slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = ClimberConstants::kP;
    slot0Configs.kI = ClimberConstants::kI; 
    slot0Configs.kD = ClimberConstants::kD;

    auto& motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 120; // turns per second --> 3 turns on the actual shaft 
    motionMagicConfigs.MotionMagicAcceleration = 80; // turns per second ^2 --> 2 tps^2

    talonFXConfigs.HardwareLimitSwitch.ForwardLimitEnable = true;
    talonFXConfigs.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
    talonFXConfigs.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = ClimberConstants::kFwdLimitAutoResetPos;
    talonFXConfigs.HardwareLimitSwitch.ReverseLimitEnable = true;
    talonFXConfigs.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true; 
    talonFXConfigs.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = ClimberConstants::kRevLimitAutoResetPos;

    m_climbMotor.GetConfigurator().Apply(talonFXConfigs, 50_ms); 

}

// This method will be called once per scheduler run
void Climber::Periodic() {
}

frc2::CommandPtr Climber::SetClimberPos(units::turn_t targetPos){
    return RunOnce(
        [this, targetPos] {
            this->m_climbMotor.SetControl(this->m_rotMotMagic.WithPosition(targetPos));
        }
    );
};

frc2::CommandPtr Climber::StopMotors() {
    return RunOnce([this] {
        this->m_climbMotor.Set(0.0);
    });
}

// enter PID Hold at position
frc2::CommandPtr Climber::StopClimb(){
    return RunOnce(
        [this] {
            this->m_climbMotor.SetControl(this->m_rotMotMagic.WithPosition(this->m_climbMotor.GetPosition().GetValue()));
        }
    );
};

frc2::CommandPtr Climber::RaiseClimber(double pos) {
    return SetClimberPos(units::turn_t(pos));
        // .AndThen(frc2::WaitUntilCommand([this] {
        //     return this->m_climbMotor.GetForwardLimit().GetValue() == ctre::phoenix6::signals::ForwardLimitValue::ClosedToGround; 
        // }).ToPtr()); 
}

frc2::CommandPtr Climber::LowerClimber(double pos) {
    return SetClimberPos(units::turn_t(pos));
        // .AndThen(frc2::WaitUntilCommand([this] {
        //     return this->m_climbMotor.GetReverseLimit().GetValue() == ctre::phoenix6::signals::ReverseLimitValue::ClosedToGround; 
        // }).ToPtr()); 
}
