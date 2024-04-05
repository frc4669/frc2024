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
    frc4669::ConfigureMotor(m_wallMotor, true);
    
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
    slot0Configs.kP = WallConstants::kP;
    slot0Configs.kI = WallConstants::kI;
    slot0Configs.kD = WallConstants::kD;

    auto& motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 120; // turns per second --> 3 turns on the actual shaft 
    motionMagicConfigs.MotionMagicAcceleration = 80; // turns per second ^2 --> 2 tps^2

    talonFXConfigs.HardwareLimitSwitch.ForwardLimitEnable = true;
    talonFXConfigs.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
    talonFXConfigs.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = WallConstants::kFwdLimitAutoResetPos;
    talonFXConfigs.HardwareLimitSwitch.ReverseLimitEnable = true;
    talonFXConfigs.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true; 
    talonFXConfigs.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = WallConstants::kFwdLimitAutoResetPos;

    m_wallMotor.GetConfigurator().Apply(talonFXConfigs, 50_ms); 

    // frc::SmartDashboard::PutNumber("P_climber", m_P); 
    // frc::SmartDashboard::PutNumber("I_climber", m_I);
    // frc::SmartDashboard::PutNumber("D_climber", m_D);
}

// This method will be called once per scheduler run
void Wall::Periodic() {
    // ctre::phoenix6::configs::TalonFXConfiguration talonFXConfigs{};
    // bool configChanged = false;

    // double newP = frc::SmartDashboard::GetNumber("P_climber", 0);
    // double newD = frc::SmartDashboard::GetNumber("I_climber", 0);
    // double newI = frc::SmartDashboard::GetNumber("D_climber", 0);

    // if (newP != m_P) {
    //     this->m_P = newP; 
    //     talonFXConfigs.Slot0.kP = newP; 
    //     configChanged = true;
    // }
    // if (newD != m_D) {
    //     this->m_D = newD; 
    //     talonFXConfigs.Slot0.kD = newD; 
    //     configChanged = true;
    // }
    // if (newI != m_I) {
    //     this->m_I = newI; 
    //     talonFXConfigs.Slot0.kI = newI; 
    //     configChanged = true;
    // }
    // if (configChanged) m_wallMotor.GetConfigurator().Apply(talonFXConfigs, 50_ms); 
}

frc2::CommandPtr Wall::SetWallPos(units::turn_t targetPos){
    return RunOnce(
        [this, targetPos] {
            this->m_wallMotor.SetControl(this->m_rotMotMagic.WithPosition(targetPos));
        }
    );
};

frc2::CommandPtr Wall::RaiseWall(double pos) {
    return SetWallPos(units::turn_t(pos))
        .AndThen(frc2::WaitUntilCommand([this] {
            return this->m_wallMotor.GetForwardLimit().GetValue() == ctre::phoenix6::signals::ForwardLimitValue::ClosedToGround; 
        }).ToPtr()); 
}

frc2::CommandPtr Wall::LowerWall(double pos) {
    return SetWallPos(units::turn_t(pos))
        .AndThen(frc2::WaitUntilCommand([this] {
            return this->m_wallMotor.GetReverseLimit().GetValue() == ctre::phoenix6::signals::ReverseLimitValue::ClosedToGround; 
        }).ToPtr()); 
}

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