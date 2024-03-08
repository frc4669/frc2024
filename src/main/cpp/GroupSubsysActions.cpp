

#include "GroupSubsysActions.h"

frc2::CommandPtr GroupSubsysActions::StopAllSupSubsys(Intake *intake, Shooter *shooter, Hand *hand) {
    return frc2::cmd::Parallel(
        hand->StopHand(),
        shooter->StopMotors(),
        intake->StopFeeder()
    ).AndThen(
        intake->StopIntake()
    );
}

frc2::CommandPtr GroupSubsysActions::NoteHandOff(Intake *intake, Shooter *shooter, Hand *hand) {
    return frc2::cmd::Sequence(
        hand->SetElevPos(-92),
        hand->SetWristPos(0),
        shooter->Shoot(0.2),
        frc2::cmd::Parallel(
            intake->StartFeeder(0.3),
            hand->Intake()
        ),
        shooter->StopMotors()
    );
}


