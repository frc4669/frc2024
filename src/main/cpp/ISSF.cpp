

#include "ISSF.h"

frc2::CommandPtr ISSF::NoteHandOff(Intake *intake, Shooter *shooter, Hand *hand) {
    return frc2::cmd::Sequence(
        hand->RaiseHand(-92),
        hand->HandTurn(0),
        shooter->Shoot(0.2),
        frc2::cmd::Parallel(
            intake->RunFeeder(0.3),
            hand->Intake()
        ),
        shooter->StopMotors()
    );
}