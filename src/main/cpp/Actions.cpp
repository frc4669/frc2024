

#include "Actions.h"

frc2::CommandPtr Actions::StopAllMotorsThatYouWouldWant(Intake *intake, Shooter *shooter, Hand *hand) {
    return frc2::cmd::Parallel(
        hand->StopHand(),
        shooter->StopMotors(),
        intake->StopFeeder()
    ).AndThen(
        intake->StopIntake()
    );
}

frc2::CommandPtr Actions::StowHand(Hand* hand) {
    return frc2::cmd::Sequence(
        hand->StopHand(),
        hand->SetWristPos(OperatorConstants::wristStowPos),
        hand->SetElevPos(OperatorConstants::elevStowPos)
    );
}

frc2::CommandPtr Actions::NoteHandOff(Intake *intake, Shooter *shooter, Hand *hand) {
    return frc2::cmd::Sequence(
        hand->SetElevPos(OperatorConstants::elevHandoffPos),
        hand->SetWristPos(OperatorConstants::wristHandoffPos),
        shooter->Shoot(OperatorConstants::shooterHandoffSpeed),
        frc2::cmd::Parallel(
            intake->StartFeeder(OperatorConstants::feederSpeed),
            hand->Intake()
        ),
        shooter->StopMotors()
    );
}

frc2::CommandPtr Actions::IntakeNote(Intake *intake) {
    return frc2::cmd::Sequence(
        intake->StartIntake(OperatorConstants::intakeSpeed),
        intake->StartFeeder(OperatorConstants::feederSpeed),
        frc2::cmd::Wait(0.02_s), // wait 1 secheduler loop run for the feeder to spin up
        intake->WaitUntilFeederCollision(), 
        intake->StopFeeder(), 
        intake->StopIntake()
    );
}

frc2::CommandPtr Actions::Shoot(Intake* intake, Shooter *shooter, Hand *hand) {
    return frc2::cmd::Sequence(
        StowHand(hand), 
        shooter->Shoot(OperatorConstants::shooterShootSpeed),
        frc2::cmd::Wait(0.05_s), //wait for spun up
        intake->StartFeeder(OperatorConstants::feederSpeed)
    );
}

frc2::CommandPtr Actions::PlaceAmp(Hand *hand) {
    return frc2::cmd::Sequence(
        hand->SetElevPos(OperatorConstants::elevAmpPos),
        hand->SetWristPos(OperatorConstants::wristAmpPos), 
        // add rotate note
        hand->Place()
    );
}

frc2::CommandPtr Actions::Climb(Climber *climber, Hand *hand) {
    return frc2::cmd::Sequence(
        hand->SetWristPos(OperatorConstants::wristClimbPos),
        hand->SetElevPos(OperatorConstants::elevClimbPos),
        climber->SetClimberPos(OperatorConstants::climbPos), 
        frc2::cmd::Wait(1_s), // wait 1 sec  
        climber->WaitUntillClimberHitHardStop(),
        climber->StopClimb()
    );
}

frc2::CommandPtr Actions::PlaceTrap(Climber *climber, Intake *intake, Shooter *shooter, Hand* hand) {
    return frc2::cmd::Sequence(
        Climb(climber, hand), 
        frc2::cmd::WaitUntil([climber] {return climber->IsClimberAtBottom();}),
        NoteHandOff(intake, shooter, hand),
        hand->SetElevPos(OperatorConstants::elevTrapPos), 
        hand->SetWristPos(OperatorConstants::wristTrapPos),
        // add rotate note
        hand->Place()
    );
}
