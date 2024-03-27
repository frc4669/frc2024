// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

#include <units/velocity.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/acceleration.h>
#include <units/time.h>
#include <units/voltage.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/math.h>

namespace Global {
  constexpr bool DEBUG = true;
}

namespace OperatorConstants {
  constexpr int kIsDefRobotTurnInPlace = true;

  constexpr int kDriverControllerPort = 0;
  constexpr int kOperatorControllerPort = 1;
  // Joystick sensitivity
  constexpr double kTurningSpeedMutiplier = 0.20;
  constexpr double kTurningBoostMutiplier = 0.5;
  constexpr double kBoostSpeedMutiplizer = 0.70;
  constexpr double kSpeedMutiplier = 0.35;

  constexpr double intakeSpeed = 0.5; // percent
  constexpr double feederSpeed = 0.3; 

  // hand stow
  constexpr double wristStowPos = 0; 
  constexpr double elevStowPos = 0; 
  constexpr double wristEnsureZeroPercent = 0.2;
  constexpr double elevEnsureZeroPercent = 0.2;

  // shooting
  constexpr double elevShootPos = 0; 
  constexpr double shooterShootSpeed = 0.98;//0.55;

  // handoff
  constexpr double elevHandoffPos = -93; 
  constexpr double shooterHandoffSpeed = 0.1;
  constexpr double wristHandoffPos = 5; 

  // amp
  constexpr double wristAmpPos = -51; 
  constexpr double elevAmpPos = -100; 

  // climbing
  // cm/rot * cm * gearrot/shaftrot
  constexpr double climbReset = 4.5*5.5 * 40;
  constexpr double climbPos = 300;
  constexpr double wristClimbPos = 0;
  constexpr double elevClimbPos = 0;

  // Trap 
  constexpr double elevTrapPos = -92;
  constexpr double wristTrapPos = -70;

}  // namespace OperatorConstants

namespace CAN{
    constexpr int kShooterTop = 41;
    constexpr int kShooterBottom = 42;
    constexpr int kLeftMainMotor = 21;
    constexpr int kLeftSecondaryMotor = 22;
    constexpr int kRightMainMotor = 11;
    constexpr int kRightSecondaryMotor = 12;

    constexpr int kFeederMotor = 31;
    constexpr int kIntakeMotor = 32;

    constexpr int kHandTopMotor = 51;
    constexpr int kHandBottomMotor = 52;
    constexpr int kHandRotationMotor = 53;
    constexpr int kHandElevatorMotor = 54;

    constexpr int kClimbMotor2 = 61;
    constexpr int kClimbMotor1 = 62;
}

namespace DIO {
  constexpr int kHandFwdLimit = 8;
  constexpr int kHandRevLimit = 9;
}

namespace DriveConstants { 
  // Feedforward gains
  constexpr auto ks = 0.000000001_V;
  constexpr auto kv = 0.000000001_V * 1_s / 1_m;
  constexpr auto ka = 0.000000001_V * 1_s * 1_s / 1_m;
  // PID controller gains
  constexpr double kp = 0.0001;
  constexpr double ki = 0;
  constexpr double kd = 0;
  // Physical parameters
  constexpr auto kTrackWidth = 20.75_in;
  constexpr double kGearRatio = 11.25;
  constexpr auto kWheelCircumference = 0.1524_m * 3.141592;
  constexpr auto kMetersPerTick = kWheelCircumference / (units::angle::turn_t(2048) * kGearRatio);
}

namespace FeederConstants {
  constexpr units::turns_per_second_t kLowestNominalVelocity = units::turns_per_second_t(20); 
}

namespace ClimberConstants {
  constexpr double kP = 0.01;
  constexpr double kI = 0; 
  constexpr double kD = 0.0;
}

namespace ElevatorConstants {
  constexpr double kMotorToShaftRatio = (1.0/30.0) * (16.0/22.0); 
  constexpr double kP = 0.2;//0.15;
  constexpr double kI = 0;
  constexpr double kD = 0.0; 
}

namespace WristConstants {
  constexpr double kP = 0.03;
  constexpr double kI = 0;
  constexpr double kD = 0.0; 
}

namespace HandConstants {
  constexpr double kP = 0.05;
  constexpr double kI = 0;
  constexpr double kD = 0.1;

  constexpr double kRampRate = 1; // seconds for a motor to go from 0 to full

  constexpr double kNoteUpPercent = 0.1; 
  constexpr double kNoteDownPercent = -0.2;
}