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

    constexpr int kIntakeMotor = 0;

    constexpr int kHandTopMotor = 51;
    constexpr int kHandBottomMotor = 52;

    constexpr int kClimbMotor2 = 61;
    constexpr int kClimbMotor1 = 62;
}
