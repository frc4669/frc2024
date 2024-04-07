// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>
#include <frc2/command/button/POVButton.h>
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
  m_autoChooser.AddOption("Two Note Mobility", m_aShootAndMobility.get()); 

  frc::SmartDashboard::PutData("Auto Chooser", &m_autoChooser);

  frc::SmartDashboard::PutNumber("Auto Start Delay", 0);

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

  // m_driverController.LeftBumper().WhileTrue(m_shooter.SysIdQuasistatic(frc2::sysid::Direction::kReverse));
  // m_driverController.RightBumper().WhileTrue(m_shooter.SysIdQuasistatic(frc2::sysid::Direction::kForward));
  // m_driverController.Y().WhileTrue(m_shooter.SysIdDynamic(frc2::sysid::Direction::kForward));
  // m_driverController.A().WhileTrue(m_shooter.SysIdDynamic(frc2::sysid::Direction::kReverse));

  // operator bindings
  m_operatorController.A().OnTrue(Actions::IntakeNote(&m_intake));
  m_operatorController.Y().OnTrue(Actions::Shoot(&m_intake, &m_shooter, &m_hand));
  // m_operatorController.Y().OnFalse(m_shooter.StopMotors())
  m_operatorController.X().OnTrue(Actions::NoteHandOff(&m_intake, &m_shooter, &m_hand));
  m_operatorController.B().OnTrue(Actions::GoToAmpPos(&m_hand));
  m_operatorController.B().OnTrue(Actions::AltGoToAmpPos(&m_hand));
  m_operatorController.RightTrigger().OnTrue(Actions::AltPlaceAmp(&m_hand));
  m_operatorController.RightTrigger().OnTrue(m_hand.Place());
  m_operatorController.LeftTrigger().OnTrue(Actions::StowHand(&m_hand));
  m_operatorController.LeftBumper().OnTrue(Actions::StopAllMotorsThatYouWouldWant(&m_intake, &m_shooter, &m_hand));

  frc2::POVButton(&m_operatorController, JoyStickConstants::kPovLeftAngle).OnTrue(m_shooter.YeetStuckNote(OperatorConstants::shooterYeetStuckNotePercent));
  frc2::POVButton(&m_operatorController, JoyStickConstants::kPovRightAngle).OnTrue(Actions::ShootCrossField(&m_intake, &m_shooter, &m_hand));
  // wall controls
  frc2::POVButton(&m_operatorController, JoyStickConstants::kPovUpAngle).OnTrue(m_wall.SetWallPos(OperatorConstants::wallUpPos));
  frc2::POVButton(&m_operatorController, JoyStickConstants::kPovDownAngle).OnTrue(m_wall.SetWallPos(OperatorConstants::wallDownPos));


  // m_operatorController.Y().WhileTrue(m_shooter.ShootVel(units::turns_per_second_t(-50)));

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
  frc::SmartDashboard::PutBoolean("C Raise Climber", false);
  frc2::Trigger([this] {
    return frc::SmartDashboard::GetBoolean("C Raise Climber", false);
  }).OnTrue(Actions::ClimberUp(&m_climber, &m_hand)); 
  frc2::Trigger([this] {
    return frc::SmartDashboard::GetBoolean("C Raise Climber", false);
  }).OnFalse(m_climber.StopClimb()); 

  frc::SmartDashboard::PutBoolean("C Lower Climber", false);
  frc2::Trigger([this] {
    return frc::SmartDashboard::GetBoolean("C Lower Climber", false);
  }).OnTrue(Actions::ClimberDown(&m_climber));

  frc::SmartDashboard::PutBoolean("C Stop Climber", false);
  frc2::Trigger([this] {
    return frc::SmartDashboard::GetBoolean("C Stop Climber", false);
  }).OnFalse(frc2::cmd::Sequence(
    m_climber.StopClimb(),
    m_climber.StopMotors()
  ));



  // frc::SmartDashboard::PutBoolean("C Place Trap", false);
  //   frc2::Trigger([this] {
  //   return frc::SmartDashboard::GetBoolean("C Place Trap", false);
  // }).OnTrue(Actions::PlaceTrap(&m_climber, &m_intake, &m_shooter, &m_hand)); 

}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  // return autos::ExampleAuto(&m_subsystem);
  return m_autoChooser.GetSelected();
  
}
