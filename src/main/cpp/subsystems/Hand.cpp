// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Hand.h"
#include "frc4669.h"
#include <frc/smartdashboard/SmartDashboard.h>

Hand::Hand(): 
    topPID(topMotor.GetPIDController()), 
    bottomPID(bottomMotor.GetPIDController()),
    topEncoder(topMotor.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42)),
    bottomEncoder(bottomMotor.GetEncoder(rev::SparkMaxRelativeEncoder::Type::kHallSensor, 42))
{
    frc4669::ConfigureRevMotor(topMotor, false);
    frc4669::ConfigureRevMotor(bottomMotor, true);
    frc4669::ConfigureMotor(rotMotor, false);

    rotMotMagic.Slot = 0;    
    ctre::phoenix6::configs::TalonFXConfiguration talonFXConfigs{};

    // set slot 0 gains
    auto& slot0Configs = talonFXConfigs.Slot0;
    // PID runs on position
    slot0Configs.kP = this->P_rot;
    slot0Configs.kI = this->I_rot;
    slot0Configs.kD = this->D_rot;

    // set Motion Magic settings
    auto& motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 12; // turns per second  
    motionMagicConfigs.MotionMagicAcceleration = 25; // turns per second ^2

    rotMotor.GetConfigurator().Apply(talonFXConfigs, 50_ms); 
    

    unviPID.SetTolerance(0.5);

    // bottomPID.SetP(0);
    // bottomPID.SetI(0);
    // bottomPID.SetD(0);
    // bottomPID.SetOutputRange(minOutCur, maxOutCur); 
    bottomMotor.SetClosedLoopRampRate(rampRate);
    // topPID.SetP(0); 
    // topPID.SetI(0); 
    // topPID.SetD(0); 
    // topPID.SetOutputRange(minOutCur, maxOutCur);
    topMotor.SetClosedLoopRampRate(rampRate);



    frc::SmartDashboard::PutNumber("P", P); 
    frc::SmartDashboard::PutNumber("I", I);
    frc::SmartDashboard::PutNumber("D", D); 
    frc::SmartDashboard::PutNumber("Pr", P_rot); 
    frc::SmartDashboard::PutNumber("Ir", I_rot);
    frc::SmartDashboard::PutNumber("Dr", D_rot); 
    frc::SmartDashboard::PutNumber("TargetRot", targetRot);
    frc::SmartDashboard::PutNumber("TargetTurn", targetTurn);
    frc::SmartDashboard::PutBoolean("UPDATE", false);
    frc::SmartDashboard::PutNumber("MinOutCur", minOutCur); 
    frc::SmartDashboard::PutNumber("MaxOutCur", maxOutCur);
    frc::SmartDashboard::PutNumber("Ramp Rate", rampRate);
}

// This method will be called once per scheduler run
void Hand::Periodic() {
    frc::SmartDashboard::PutNumber("Pos Avg", (topEncoder.GetPosition() + bottomEncoder.GetPosition())/2);
    frc::SmartDashboard::PutBoolean("PID At Set point", unviPID.AtSetpoint());

    if (frc::SmartDashboard::GetBoolean("UPDATE", false)) {
        double newP_rot = frc::SmartDashboard::GetNumber("Pr", 0);
        double newI_rot = frc::SmartDashboard::GetNumber("Ir", 0);
        double newD_rot = frc::SmartDashboard::GetNumber("Dr", 0); 
        ctre::phoenix6::configs::TalonFXConfiguration talonFXConfigs{};
        bool rotPIDchanged = false;

        double newP = frc::SmartDashboard::GetNumber("P", 0);
        double newI = frc::SmartDashboard::GetNumber("I", 0);
        double newD = frc::SmartDashboard::GetNumber("D", 0); 
        double newTargetRot = frc::SmartDashboard::GetNumber("TargetRot", 0); 
        double newTargetTurn = frc::SmartDashboard::GetNumber("TargetTurn", 0); 
        double newMinCur = frc::SmartDashboard::GetNumber("MinOutCur", 0); 
        double newMaxCur = frc::SmartDashboard::GetNumber("MaxOutCur", 0); 
        double newRampRate = frc::SmartDashboard::PutNumber("RampRate", 0);
        if (newRampRate != rampRate) {
            this->rampRate = newRampRate;
            topMotor.SetClosedLoopRampRate(rampRate);
            bottomMotor.SetClosedLoopRampRate(rampRate);
        }
        // if (newMinCur != minOutCur) {
        //     this->minOutCur = newMinCur; 
        //     bottomPID.SetOutputRange(minOutCur, maxOutCur); 
        //     topPID.SetOutputRange(minOutCur, maxOutCur);
        // }
        if (newP != P) {
            this->P = newP; 
            unviPID.SetP(newP); 
            // topPID.SetP(this->P); 
            // bottomPID.SetP(this->P);
        }
        if (newI != I) {
            this->I = newI; 
            unviPID.SetI(newI); 
        }
        if (newD != D) {
            this->D = newD; 
            unviPID.SetD(newD); 
        }
        if (newP_rot != P_rot) {
            this->P_rot = newP_rot; 
            talonFXConfigs.Slot0.kP = newP_rot;
            rotPIDchanged = true;
        }
        if (newI_rot != I_rot) {
            this->I_rot = newI_rot; 
            talonFXConfigs.Slot0.kI = newI_rot;
            rotPIDchanged = true;
        }
        if (newD_rot != D_rot) {
            this->D_rot = newD_rot; 
            talonFXConfigs.Slot0.kD = newD_rot;
            rotPIDchanged = true;
        }
        if (newTargetRot != targetRot) {
            this->targetRot = newTargetRot;
        }
        if (newTargetTurn != targetTurn) {
            this->targetTurn = newTargetTurn;
        }
        if (rotPIDchanged) rotMotor.GetConfigurator().Apply(talonFXConfigs, 50_ms); 
    }

    this->rotMotor.SetControl(this->rotMotMagic.WithPosition(units::turn_t(this->targetTurn))); 

}

void Hand::EnsureInvert(bool inverted) {
    bottomMotor.Follow(topMotor, inverted);
}

frc2::CommandPtr Hand::Intake () {
    return RunOnce([this] { this->EnsureInvert(false); }).AndThen( 
        Run(
            [this] {
                topMotor.Set(0.1);
            }
        ).WithTimeout(500_ms)
        .AndThen(StopHand())
    );
}

frc2::CommandPtr Hand::Place () {
    return RunOnce([this] { this->EnsureInvert(false); }).AndThen( 
        Run(
            [this] {
                topMotor.Set(-0.05);
            }
        ).WithTimeout(500_ms)
        .AndThen(StopHand())
    );
}

// turns hand with PID
frc2::CommandPtr Hand::HandTurn () {
    return Run(
        [this] {
            
        }
    );
}

// top and bottom needs to be going the same direction
// current test config INVERT: bottom needs to be negated 
frc2::CommandPtr Hand::TurnNote () {
    double intialPostion = (topEncoder.GetPosition() + bottomEncoder.GetPosition())/2; 
    this->unviPID.Reset(); 
    this->unviPID.SetSetpoint(intialPostion + targetRot);
    return RunOnce([this] { this->EnsureInvert(true); }).AndThen(
        Run(
            [this, intialPostion] {
                GoToSetPoint(intialPostion + targetRot); 
            }
        ).Until([this] { return unviPID.AtSetpoint(); })
        .AndThen(StopHand())
    );
}
frc2::CommandPtr Hand::StopHand (){
    return RunOnce([this] { this->EnsureInvert(true); }).AndThen(
        RunOnce(
            [this] {
                topMotor.Set(0.0);
                bottomMotor.Set(0.0);
            }
        )
    );
}

void Hand::GoToSetPoint(double setpoint) {
    units::volt_t output = units::volt_t(unviPID.Calculate((topEncoder.GetPosition() + bottomEncoder.GetPosition())/2, setpoint));
    frc::SmartDashboard::PutNumber("PID _OUT", output.value()); 
    
    topMotor.SetVoltage(output);
}
