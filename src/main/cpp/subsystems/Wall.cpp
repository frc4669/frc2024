// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Wall.h"
#include "frc4669.h"
#include <frc2/command/CommandPtr.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/WaitUntilCommand.h>
#include <units/voltage.h>
#include "Actions.h"

Wall::Wall() {
    frc4669::ConfigureMotor(m_wallMotor, false);
    
    // motor config
    ctre::phoenix6::configs::HardwareLimitSwitchConfigs fwdLimitConfig {};

    // motion magic
    m_rotMotMagic.Slot = 0;    
    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfigs{};

    auto& slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = WallConstants::kP;
    slot0Configs.kI = WallConstants::kI;
    slot0Configs.kD = WallConstants::kD;

    auto& motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 120; // turns per second --> 3 turns on the actual shaft 
    motionMagicConfigs.MotionMagicAcceleration = 80; // turns per second ^2 --> 2 tps^2

    talonFXConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable =true;
    talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold = WallConstants::kLimitPos;
    talonFXConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;
    talonFXConfigs.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true; 
    talonFXConfigs.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = WallConstants::kLimitPos;
    talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;

    m_wallMotor.GetConfigurator().Apply(talonFXConfigs, 50_ms); 

}

// This method will be called once per scheduler run
void Wall::Periodic() {
    
}

frc2::CommandPtr Wall::SetWallPos(units::turn_t targetPos){
    return RunOnce(
        [this, targetPos] {
            this->m_wallMotor.SetControl(this->m_rotMotMagic.WithPosition(targetPos));
        }
    );
};

frc2::CommandPtr Wall::StopMotors() {
    return RunOnce([this] {
        this->m_wallMotor.Set(0.0);
    });
}

// enter PID Hold at position
frc2::CommandPtr Wall::StopWall(){
    return RunOnce(
        [this] {
            this->m_wallMotor.SetControl(this->m_rotMotMagic.WithPosition(this->m_wallMotor.GetPosition().GetValue()));
        }
    );
};