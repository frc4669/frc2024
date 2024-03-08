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
  m_drivetrain.SetDefaultCommand(m_drivetrain.DefaultDriveCommand(
    [this] { return -m_driverController.GetLeftY() * OperatorConstants::kSpeedMutiplier; },
    [this] { return -m_driverController.GetRightX() * OperatorConstants::kTurningSpeedMutiplier; }
  ));
  
  m_driverController.X().OnTrue(Actions::IntakeNote(&m_intake));

  // operator bindings
  m_operatorController.A().OnTrue(Actions::IntakeNote(&m_intake));
  m_operatorController.Y().OnTrue(Actions::Shoot(&m_intake, &m_shooter, &m_hand));
  m_operatorController.X().OnTrue(Actions::NoteHandOff(&m_intake, &m_shooter, &m_hand));
  m_operatorController.B().OnTrue(Actions::PlaceAmp(&m_hand));

  m_operatorController.LeftTrigger().OnTrue(Actions::StowHand(&m_hand));
  m_operatorController.LeftBumper().OnTrue(Actions::StopAllMotorsThatYouWouldWant(&m_intake, &m_shooter, &m_hand));

  // smart dash board controls 
  frc::SmartDashboard::PutBoolean("C Climb", false);
  frc2::Trigger([this] {
    return frc::SmartDashboard::GetBoolean("C Climb", false);
  }).OnTrue(Actions::Climb(&m_climber, &m_hand)); 

  frc::SmartDashboard::PutBoolean("C Place Trap", false);
    frc2::Trigger([this] {
    return frc::SmartDashboard::GetBoolean("C Place Trap", false);
  }).OnTrue(Actions::PlaceTrap(&m_climber, &m_intake, &m_shooter, &m_hand)); 

}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  // return autos::ExampleAuto(&m_subsystem);
}
