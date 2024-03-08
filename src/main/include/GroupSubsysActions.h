#pragma once

#include "subsystems/Drivetrain.h"
#include "subsystems/Hand.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"

#include <frc2/command/CommandPtr.h>

namespace GroupSubsysActions {
    frc2::CommandPtr NoteHandOff(
        Intake *intake,
        Shooter *shooter, 
        Hand *hand
    );
    frc2::CommandPtr StopAllSupSubsys(Intake *intake, Shooter *shooter, Hand *hand);
}