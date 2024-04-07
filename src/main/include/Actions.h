#pragma once

#include "subsystems/Drivetrain.h"
#include "subsystems/Hand.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
#include "subsystems/Climber.h"

#include <frc2/command/CommandPtr.h>

namespace Actions {
    frc2::CommandPtr StopAllMotorsThatYouWouldWant(Intake *intake, Shooter *shooter, Hand *hand);
    frc2::CommandPtr IntakeNote(Intake *intake);
    frc2::CommandPtr StowHand(Hand* hand);
    frc2::CommandPtr Shoot(Intake* intake, Shooter *shooter, Hand *hand);
    frc2::CommandPtr ShootCrossField(Intake* intake, Shooter *shooter, Hand *hand);
    frc2::CommandPtr NoteHandOff(Intake *intake, Shooter *shooter, Hand *hand);
    frc2::CommandPtr GoToAmpPos(Hand *hand); 
    frc2::CommandPtr ClimberUp(Climber *climber, Hand *hand);
    frc2::CommandPtr ClimberDown(Climber *climber);
    frc2::CommandPtr PlaceTrap(Climber *climber, Intake *intake, Shooter *shooter, Hand* hand);

    frc2::CommandPtr AltGoToAmpPos(Hand *hand); 
    frc2::CommandPtr AltPlaceAmp(Hand *hand);
}

// all autos require all subsystems 
namespace ActionsAutos {
    frc2::CommandPtr AutoStartTimer();  
    frc2::CommandPtr DoNothingAuto(Drivetrain *drivetrain, Hand *hand, Intake *intake, Shooter *shooter, Climber *Climber);
    frc2::CommandPtr ShootOnly(Drivetrain *drivetrain, Hand *hand, Intake *intake, Shooter *shooter, Climber *Climber);
    frc2::CommandPtr ShootAndMobility(Drivetrain *drivetrain, Hand *hand, Intake *intake, Shooter *shooter, Climber *Climber);
    frc2::CommandPtr TwoNoteMobility(Drivetrain *drivetrain, Hand *hand, Intake *intake, Shooter *shooter, Climber *Climber);
}