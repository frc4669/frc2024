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
  //////////
  // AUTO //
  //////////
  m_autoChooser.SetDefaultOption("Do Nothing Auto", m_aDoNothing.get()); 
  m_autoChooser.AddOption("Shoot Only", m_aShootonly.get());
  m_autoChooser.AddOption("Shoot and mobility", m_aShootAndMobility.get()); 

  // Configure your trigger bindings here

  // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  m_drivetrain.SetDefaultCommand(m_drivetrain.DefaultDriveCommand(
    [this] { return -m_driverController.GetLeftY(); },
    [this] { return -m_driverController.GetRightX(); }
  ));
  
  m_driverController.X().OnTrue(Actions::IntakeNote(&m_intake));

  m_driverController.RightTrigger().OnTrue(m_drivetrain.SetSpeedMutiplier(OperatorConstants::kBoostSpeedMutiplizer));
  m_driverController.RightTrigger().OnFalse(m_drivetrain.SetSpeedMutiplier(OperatorConstants::kSpeedMutiplier));

  m_driverController.LeftTrigger().OnTrue(m_drivetrain.SetTurnMutiplier(OperatorConstants::kTurningBoostMutiplier));
  m_driverController.LeftTrigger().OnFalse(m_drivetrain.SetTurnMutiplier(OperatorConstants::kTurningSpeedMutiplier));

  // operator bindings
  m_operatorController.A().OnTrue(Actions::IntakeNote(&m_intake));
  m_operatorController.Y().OnTrue(Actions::Shoot(&m_intake, &m_shooter, &m_hand));
  m_operatorController.X().OnTrue(Actions::NoteHandOff(&m_intake, &m_shooter, &m_hand));
  m_operatorController.B().OnTrue(Actions::GoToAmpPos(&m_hand));
  m_operatorController.RightTrigger().OnTrue(m_hand.Place());
  m_operatorController.LeftTrigger().WhileTrue(Actions::StowHand(&m_hand));
  m_operatorController.LeftBumper().WhileTrue(Actions::StopAllMotorsThatYouWouldWant(&m_intake, &m_shooter, &m_hand));

  // m_operatorController.X().OnTrue(
  //   frc2::cmd::Sequence(
  //   m_hand.SetElevPos(-95), 
  //   m_hand.SetWristPos(-30))
  // );

  // m_operatorController.A().OnTrue(m_hand.Intake());
  // m_operatorController.Y().OnTrue(m_hand.Place());

  // m_operatorController.RightTrigger().OnTrue(frc2::cmd::RunOnce([this] {m_hand.GoToSetPoint(); }));
  // m_operatorController.RightBumper().OnTrue(m_hand.TurnNote(5));



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

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  // return autos::ExampleAuto(&m_subsystem);
  return m_autoChooser.GetSelected();
  
}
