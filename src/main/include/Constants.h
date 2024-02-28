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

namespace OperatorConstants {

inline constexpr int kDriverControllerPort = 0;
  // Joystick sensitivity
  constexpr double kTurningSpeedMutiplier = 0.7;
  constexpr double kSpeedMutiplier = 0.5;

}  // namespace OperatorConstants

namespace CAN{
    constexpr int kFeederMotor = 41;
    constexpr int kShooterMotor = 42;
    constexpr int kLeftMainMotor = 11;
    constexpr int kLeftSecondaryMotor = 12;
    constexpr int kRightMainMotor = 21;
    constexpr int kRightSecondaryMotor = 22;

    constexpr int kIntakeMotor = 42;

    constexpr int kHandTopMotor = 51;
    constexpr int kHandBottomMotor = 52;

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