// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Hand.h"
#include "frc4669.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/WaitUntilCommand.h>

Hand::Hand():
    m_topEncoder(m_topMotor.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42)),
    m_bottomEncoder(m_bottomMotor.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42))
 {
    frc4669::ConfigureRevMotor(m_topMotor, false);
    frc4669::ConfigureRevMotor(m_bottomMotor, true);
    frc4669::ConfigureMotor(m_wristMotor, false);
    frc4669::ConfigureMotor(m_elevMotor,false);
  
    { // rot motor special config
    m_wristMotMagic.Slot = 0;    
    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfigs{};
    // set slot 0 gains
    auto& slot0Configs = talonFXConfigs.Slot0;
    // PID runs on position
    slot0Configs.kP = this->m_P_wri;
    slot0Configs.kI = this->m_I_wri;
    slot0Configs.kD = this->m_D_wri;

    // set Motion Magic settings
    auto& motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 12; // turns per second  
    motionMagicConfigs.MotionMagicAcceleration = 25; // turns per second ^2

    talonFXConfigs.HardwareLimitSwitch.ForwardLimitEnable = true;
    talonFXConfigs.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
    talonFXConfigs.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0;
    talonFXConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;

    m_wristMotor.GetConfigurator().Apply(talonFXConfigs, 50_ms); 
    
    }

    {// elev motor special config
    m_elevMotMagic.Slot = 0;    
    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfigs{};
    // set slot 0 gains
    auto& slot0Configs = talonFXConfigs.Slot0;
    // PID runs on position
    slot0Configs.kP = this->m_P_elev;
    slot0Configs.kI = this->m_I_elev;
    slot0Configs.kD = this->m_D_elev;

    // set Motion Magic settings
    auto& motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 12; // turns per second  
    motionMagicConfigs.MotionMagicAcceleration = 25; // turns per second ^2

    talonFXConfigs.HardwareLimitSwitch.ForwardLimitEnable = true;
    talonFXConfigs.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
    talonFXConfigs.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 0;

    talonFXConfigs.HardwareLimitSwitch.ForwardLimitEnable = true;
    talonFXConfigs.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
    talonFXConfigs.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = -130;

    m_elevMotor.GetConfigurator().Apply(talonFXConfigs, 50_ms); 
    }

    m_unviPID.SetTolerance(0.5);

    m_bottomMotor.SetClosedLoopRampRate(m_rampRate);
    m_topMotor.SetClosedLoopRampRate(m_rampRate);



    frc::SmartDashboard::PutNumber("m_P_hand", m_P_hand); 
    frc::SmartDashboard::PutNumber("m_I_hand", m_I_hand);
    frc::SmartDashboard::PutNumber("m_D_hand", m_D_hand); 
    frc::SmartDashboard::PutNumber("Pr", m_P_wri); 
    frc::SmartDashboard::PutNumber("Ir", m_I_wri);
    frc::SmartDashboard::PutNumber("Dr", m_D_wri); 
    frc::SmartDashboard::PutNumber("Pe", m_P_elev); 
    frc::SmartDashboard::PutNumber("Ie", m_I_elev);
    frc::SmartDashboard::PutNumber("De", m_D_elev); 
    frc::SmartDashboard::PutBoolean("UPDATE", false);

    frc::SmartDashboard::PutNumber("TargetTurn", targetTurn); 
    frc::SmartDashboard::PutBoolean("ElevFinished", false);
    frc::SmartDashboard::PutBoolean("WristFinished", false);
}

// This method will be called once per scheduler run
void Hand::Periodic() {
    frc::SmartDashboard::PutNumber("Pos Avg", (m_topEncoder.GetPosition() + m_bottomEncoder.GetPosition())/2);
    frc::SmartDashboard::PutBoolean("PID At Set point", m_unviPID.AtSetpoint());

    if (frc::SmartDashboard::GetBoolean("UPDATE", false)) {
        double newP_rot = frc::SmartDashboard::GetNumber("Pr", 0);
        double newI_rot = frc::SmartDashboard::GetNumber("Ir", 0);
        double newD_rot = frc::SmartDashboard::GetNumber("Dr", 0); 
        ctre::phoenix6::configs::TalonFXConfiguration rotMotorConfigs{};
        bool rotPIDchanged = false;

        double newP_elev = frc::SmartDashboard::GetNumber("Pe", 0);
        double newI_elev = frc::SmartDashboard::GetNumber("Ie", 0);
        double newD_elev = frc::SmartDashboard::GetNumber("De", 0); 
        ctre::phoenix6::configs::TalonFXConfiguration elevMotorConfigs{};
        bool elevPIDchanged = false;

        double newTargetTurn = frc::SmartDashboard::GetNumber("TargetTurn", 0); 


        double newP = frc::SmartDashboard::GetNumber("m_P_hand", 0);
        double newI = frc::SmartDashboard::GetNumber("m_I_hand", 0);
        double newD = frc::SmartDashboard::GetNumber("m_D_hand", 0); 
        double newRampRate = frc::SmartDashboard::PutNumber("RampRate", 0);
        if (newTargetTurn != targetTurn) {
            this->targetTurn = newTargetTurn;  
        }
        if (newRampRate != m_rampRate) {
            this->m_rampRate = newRampRate;
            m_topMotor.SetClosedLoopRampRate(m_rampRate);
            m_bottomMotor.SetClosedLoopRampRate(m_rampRate);
        }
        if (newP != m_P_hand) {
            this->m_P_hand = newP; 
            m_unviPID.SetP(newP); 
        }
        if (newI != m_I_hand) {
            this->m_I_hand = newI; 
            m_unviPID.SetI(newI); 
        }
        if (newD != m_D_hand) {
            this->m_D_hand = newD; 
            m_unviPID.SetD(newD); 
        }
        if (newP_rot != m_P_wri) {
            this->m_P_wri = newP_rot; 
            rotMotorConfigs.Slot0.kP = newP_rot;
            rotPIDchanged = true;
        }
        if (newI_rot != m_I_wri) {
            this->m_I_wri = newI_rot; 
            rotMotorConfigs.Slot0.kI = newI_rot;
            rotPIDchanged = true;
        }
        if (newD_rot != m_D_wri) {
            this->m_D_wri = newD_rot; 
            rotMotorConfigs.Slot0.kD = newD_rot;
            rotPIDchanged = true;
        }
        if (newP_elev != m_P_elev) {
            this->m_P_elev = newP_elev; 
            elevMotorConfigs.Slot0.kP = newP_elev;
            elevPIDchanged = true;
        }
        if (newI_elev != m_I_elev) {
            this->m_I_elev = newI_elev; 
            elevMotorConfigs.Slot0.kI = newI_elev;
            elevPIDchanged = true;
        }
        if (newD_elev != m_D_elev) {
            this->m_D_elev = newD_elev; 
            elevMotorConfigs.Slot0.kD = newD_elev;
            elevPIDchanged = true;
        }
        if (elevPIDchanged) m_elevMotor.GetConfigurator().Apply(elevMotorConfigs, 50_ms); 
        if (rotPIDchanged) m_wristMotor.GetConfigurator().Apply(rotMotorConfigs, 50_ms); 
    }

     
    frc::SmartDashboard::PutNumber("velelev", units::math::abs(this->m_wristMotor.GetVelocity().GetValue()) < units::turns_per_second_t(0.1) && units::math::abs(this->m_wristMotor.GetPosition().GetValue() - units::turn_t(-92)) < 10_tr) ;
    frc::SmartDashboard::PutData(&m_unviPID);
}

void Hand::EnsureInvert(bool inverted) {
    m_bottomMotor.Follow(m_topMotor, inverted);
}

frc2::CommandPtr Hand::Intake (units::second_t timeout) {
    return RunOnce([this] { this->EnsureInvert(false); }).AndThen( 
        Run(
            [this] {
                m_topMotor.Set(0.3);
            }
        ).WithTimeout(timeout)
        .AndThen(StopHand())
    );
}

frc2::CommandPtr Hand::Place () {
    return RunOnce([this] { this->EnsureInvert(false); }).AndThen( 
        Run(
            [this] {
                m_topMotor.Set(-0.05);
            }
        ).WithTimeout(500_ms)
        .AndThen(StopHand())
    );
}

// turns hand with PID
frc2::CommandPtr Hand::SetWristPos(double pos) {
    return Run([this, pos] {
        this->m_wristMotor.SetControl(this->m_wristMotMagic.WithPosition(units::turn_t(pos)));
    })
    .Until([this, pos] {
        return units::math::abs(this->m_wristMotor.GetVelocity().GetValue()) < units::turns_per_second_t(1) 
        && units::math::abs(this->m_wristMotor.GetPosition().GetValue() - units::turn_t(pos)) < 10_tr;
    })
    .AndThen([this] {
        frc::SmartDashboard::PutBoolean("WristFinished", true);
    });
}

frc2::CommandPtr Hand::SetElevPos(double pos) {
    return Run([this, pos] {
        this->m_elevMotor.SetControl(this->m_elevMotMagic.WithPosition(units::turn_t(pos))); 
    })
    .Until([this, pos] {
        return units::math::abs(this->m_elevMotor.GetVelocity().GetValue()) < units::turns_per_second_t(1) 
        && units::math::abs(this->m_elevMotor.GetPosition().GetValue() - units::turn_t(pos)) < 10_tr;
    })
    .AndThen([this] {
        frc::SmartDashboard::PutBoolean("ElevFinished", true);

    });
}

frc2::CommandPtr Hand::EnsureElevZero(double zeroingSpeed) {
    return RunOnce(
        [this, zeroingSpeed] {this->m_elevMotor.Set(zeroingSpeed); }
    ).AndThen(
    frc2::WaitUntilCommand(
        [this] {
            return this->m_elevMotor.GetForwardLimit().GetValue() == ctre::phoenix6::signals::ForwardLimitValue::ClosedToGround;
        }).ToPtr()
    );
}

frc2::CommandPtr Hand::EnsureWristZero(double zeroingSpeed) {
    return RunOnce(
        [this, zeroingSpeed] {this->m_wristMotor.Set(zeroingSpeed); }
    ).AndThen(
    frc2::WaitUntilCommand(
        [this] {
            return this->m_wristMotor.GetForwardLimit().GetValue() == ctre::phoenix6::signals::ForwardLimitValue::ClosedToGround;
        }).ToPtr()
    );
}

// top and bottom needs to be going the same direction
// current test config INVERT: bottom needs to be negated 
frc2::CommandPtr Hand::TurnNote (double pos) {
    // pos = this->targetTurn;
    
    return RunOnce([this, pos] { 
        this->EnsureInvert(true);
        double intialPostion = (m_topEncoder.GetPosition() + m_bottomEncoder.GetPosition())/2; 
        this->m_unviPID.Reset(); 
        this->m_unviPID.SetSetpoint(intialPostion + pos); 
    }).AndThen(
        Run(
            [this] {
                GoToSetPoint(); 
            }
        ).Until([this] { return m_unviPID.AtSetpoint(); })
        .AndThen(StopHand())
    );
}

frc2::CommandPtr Hand::TurnNotePercentOutput(double output) {
    return RunOnce([this] {this->EnsureInvert(true);}).AndThen(
        Run(
        [this, output] {
            
            this->m_topMotor.Set(output); 
        }
    ).WithTimeout(0.25_s))
    .AndThen(StopHand());
}

// stop both hand motors
frc2::CommandPtr Hand::StopHand (){
    return RunOnce([this] { this->EnsureInvert(true); }).AndThen(
        RunOnce(
            [this] {
                m_topMotor.Set(0.0);
                // m_bottomMotor.Set(0.0);
            }
        )
    );
}

// go to setpoint with PID
void Hand::GoToSetPoint() {
    units::volt_t output = units::volt_t(m_unviPID.Calculate((m_topEncoder.GetPosition() + m_bottomEncoder.GetPosition())/2));
    frc::SmartDashboard::PutNumber("PID _OUT", output.value()); 
    
    m_topMotor.SetVoltage(output);
}
