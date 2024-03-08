// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include <frc2/command/RepeatCommand.h>

RobotContainer::RobotContainer() {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  // m_drivetrain.SetDefaultCommand(m_drivetrain.DefaultDriveCommand(
  //   [this] { return -m_driverController.GetLeftY() * OperatorConstants::kSpeedMutiplier; },
  //   [this] { return -m_driverController.GetRightX() * OperatorConstants::kTurningSpeedMutiplier; }
  // ));
  
  // frc2::Trigger([this] {
  //   return m_subsystem.ExampleCondition();
  // }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  // m_driverController.X().OnTrue(m_shooter.Shoot(0.25));
    
  //  m_driverController.B().OnTrue( m_shooter.StopMotors());

  // m_driverController.Y().OnTrue( m_intake.StartIntake(0.5));

  // m_driverController.X().OnTrue( m_intake.StopIntake());
  // m_driverController.A().OnTrue( m_intake.StartFeeder(0.3));

  m_driverController.B().OnTrue( GroupSubsysActions::NoteHandOff(&m_intake, &m_shooter, &m_hand));
  m_driverController.X().OnTrue(GroupSubsysActions::StopAllSupSubsys(&m_intake, &m_shooter, &m_hand));

  // m_driverController.LeftBumper().OnTrue(m_hand.SetElevPos(-20));
  // m_driverController.LeftTrigger().OnTrue(m_shooter.StopMotors());

  // m_driverController.B().OnTrue(m_climber.ZeroClimber());
  // m_driverController.Y().OnTrue(m_climber.StartClimb());
  // m_driverController.X().OnTrue(m_climber.StopClimb());


  // m_driverController.RightTrigger().OnTrue(); 
  // m_driverController.LeftBumper().OnTrue( m_hand.StopHand());


}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  // return autos::ExampleAuto(&m_subsystem);
}
