// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include "Constants.h"
#include "subsystems/Shooter.h"
#include "subsystems/Drivetrain.h"
#include "subsystems/Intake.h"
#include "subsystems/Hand.h"
#include "subsystems/Climber.h"
#include "subsystems/Wall.h"

#include "Actions.h"

#include <frc/smartdashboard/SendableChooser.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

 private:
  // Replace with CommandPS4Controller or CommandJoystick if needed
  frc2::CommandXboxController m_driverController{OperatorConstants::kDriverControllerPort};
  frc2::CommandXboxController m_operatorController {OperatorConstants::kOperatorControllerPort};

  // The robot's subsystems are defined here...
  Drivetrain m_drivetrain;
  Shooter m_shooter;
  Intake m_intake;
  Hand m_hand;
  Climber m_climber;
  Wall m_wall;

  frc::SendableChooser<frc2::Command *> m_autoChooser; 

  void ConfigureBindings();

  ///////////
  // Autos //
  ///////////
  frc2::CommandPtr m_aDoNothing {ActionsAutos::DoNothingAuto(&m_drivetrain, &m_hand, &m_intake, &m_shooter, &m_climber)};
  frc2::CommandPtr m_aShootonly {ActionsAutos::ShootOnly(&m_drivetrain, &m_hand, &m_intake, &m_shooter, &m_climber)};
  frc2::CommandPtr m_aShootAndMobility {ActionsAutos::ShootAndMobility(&m_drivetrain, &m_hand, &m_intake, &m_shooter, &m_climber)};
  frc2::CommandPtr m_aTwoNoteMobility {ActionsAutos::ShootAndMobility(&m_drivetrain, &m_hand, &m_intake, &m_shooter, &m_climber)};
};
