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
    frc2::CommandPtr NoteHandOff(Intake *intake, Shooter *shooter, Hand *hand);
    frc2::CommandPtr GoToAmpPos(Hand *hand); 
    frc2::CommandPtr Climb(Climber *climber, Hand *hand);
    frc2::CommandPtr PlaceTrap(Climber *climber, Intake *intake, Shooter *shooter, Hand* hand);
}