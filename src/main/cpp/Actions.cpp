

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
        hand->SetElevPos(OperatorConstants::elevStowPos),
        hand->EnsureElevZero(OperatorConstants::elevEnsureZeroPercent), 
        hand->EnsureWristZero(OperatorConstants::wristEnsureZeroPercent)
    );
}

frc2::CommandPtr Actions::NoteHandOff(Intake *intake, Shooter *shooter, Hand *hand) {
    return frc2::cmd::Sequence(
        hand->SetWristPos(OperatorConstants::wristHandoffPos),
        hand->SetElevPos(OperatorConstants::elevHandoffPos),
        shooter->Shoot(OperatorConstants::shooterHandoffSpeed),
        frc2::cmd::Parallel(
            intake->StartFeeder(OperatorConstants::feederSpeed),
            hand->Intake(0.5_s)
        ),
        shooter->StopMotors(),
        intake->StopFeeder()
    );
}

// intake note is actually ran twice in the function actually binded to the controller
// in order to ensure that the shooter will get a note when we press shoot
frc2::CommandPtr _IntakeNote(Intake *intake) {
    return frc2::cmd::Sequence(
        intake->StartIntake(OperatorConstants::intakeSpeed),
        intake->StartFeeder(OperatorConstants::feederSpeed),
        frc2::cmd::Wait(0.1_s), // wait 1 secheduler loop run for the feeder to spin up
        intake->WaitUntilFeederCollision(), 
        intake->StopFeeder(), 
        intake->StopIntake()
    );
}
frc2::CommandPtr Actions::IntakeNote(Intake *intake) {
    return frc2::cmd::Sequence(
        _IntakeNote(intake),
        frc2::cmd::RunOnce([] {frc::SmartDashboard::PutBoolean("Feeder Got Note", true);})//,
        // _IntakeNote(intake)
    );
}

frc2::CommandPtr Actions::Shoot(Intake* intake, Shooter *shooter, Hand *hand) {
    return frc2::cmd::Sequence(
        frc2::cmd::Parallel(
            StowHand(hand)
        ), 
        // shooter->Shoot(OperatorConstants::shooterShootSpeed),
        shooter->ShootVel(OperatorConstants::shooterTopSpeed, OperatorConstants::shooterBottomSpeed),
        frc2::cmd::Wait(0.5_s), //wait for spun up
        intake->StartFeeder(OperatorConstants::feederSpeed),
        frc2::cmd::Wait(1_s), // wait 1 sec for shoot to finish
        shooter->StopMotors(),
        intake->StopFeeder(),
        frc2::cmd::RunOnce([] {frc::SmartDashboard::PutBoolean("Feeder Got Note", false);})
    );
}

frc2::CommandPtr Actions::ShootCrossField(Intake* intake, Shooter *shooter, Hand *hand) {
    return frc2::cmd::Sequence(
        frc2::cmd::Parallel(
            StowHand(hand)
        ), 
        shooter->ShootVel(OperatorConstants::shooterCrossFieldCommonSpeed, OperatorConstants::shooterCrossFieldCommonSpeed),
        frc2::cmd::Wait(0.5_s), //wait for spun up
        intake->StartFeeder(OperatorConstants::feederSpeed),
        frc2::cmd::Wait(1_s), // wait 1 sec for shoot to finish
        shooter->StopMotors(),
        intake->StopFeeder(),
        frc2::cmd::RunOnce([] {frc::SmartDashboard::PutBoolean("Feeder Got Note", false);})
    );
}

// frc2::CommandPtr Actions::Shoot(Intake* intake, Shooter *shooter, Hand *hand) {
//     return frc2::cmd::Sequence(
//         StowHand(hand), 
//         frc2::cmd::Parallel(
//             shooter->ShootVel(units::turns_per_second_t(-50)),
//             frc2::cmd::Sequence(
//                 intake->StartFeeder(OperatorConstants::feederSpeed),
//                 frc2::cmd::Wait(1.5_s), 
//                 Actions::IntakeNote(intake)
//             )
//         )
//     );
// }

frc2::CommandPtr Actions::GoToAmpPos(Hand *hand) {
    return frc2::cmd::Sequence(
        hand->SetElevPos(OperatorConstants::elevAmpPos),
        hand->SetWristPos(OperatorConstants::wristAmpPos),
        hand->TurnNotePercentOutput(HandConstants::kNoteDownPercent),
        hand->Intake(0.1_s)
        // add rotate note
        // hand->Place()
    );
}

frc2::CommandPtr Actions::ClimberUp(Climber *climber, Hand *hand) {
    return frc2::cmd::Sequence(
        hand->SetElevPos(OperatorConstants::elevClimbPos),
        hand->SetWristPos(OperatorConstants::wristClimbPos),
        climber->RaiseClimber(OperatorConstants::climbUpPos)
    );
}

frc2::CommandPtr Actions::ClimberDown(Climber *climber) {
    return frc2::cmd::Sequence(
        climber->LowerClimber(OperatorConstants::climbDownPos)
    );
}

// frc2::CommandPtr Actions::PlaceTrap(Climber *climber, Intake *intake, Shooter *shooter, Hand* hand) {
//     return frc2::cmd::Sequence(
//         ClimberUp(climber, hand), 
//         frc2::cmd::WaitUntil([climber] {return climber->IsClimberAtBottom();}),
//         NoteHandOff(intake, shooter, hand),
//         hand->SetElevPos(OperatorConstants::elevTrapPos), 
//         hand->SetWristPos(OperatorConstants::wristTrapPos),
//         // add rotate note
//         hand->Place()
//     );
// }

// go to amp but have note be defaulted up
frc2::CommandPtr Actions::AltGoToAmpPos(Hand *hand) {
    return frc2::cmd::Sequence(
        hand->SetElevPos(OperatorConstants::elevAmpPos),
        hand->SetWristPos(OperatorConstants::wristAmpPos), 
        hand->TurnNotePercentOutput(HandConstants::kNoteDownPercent)
    );
}

frc2::CommandPtr Actions::AltPlaceAmp(Hand *hand) {
    return frc2::cmd::Sequence(
        hand->Place() 

    );
}


/////////////
/// AUTOS ///
/////////////
frc2::CommandPtr ActionsAutos::AutoStartTimer() {
    return frc2::cmd::Wait(units::second_t(frc::SmartDashboard::GetNumber("Auto Start Delay", AutoConstants::kAutoDefaultStartWaitSec)));
}

frc2::CommandPtr ActionsAutos::DoNothingAuto(Drivetrain *drivetrain, Hand *hand, Intake *intake, Shooter *shooter, Climber *Climber) {
    return frc2::cmd::None(); 
}

frc2::CommandPtr ActionsAutos::ShootOnly(Drivetrain *drivetrain, Hand *hand, Intake *intake, Shooter *shooter, Climber *Climber) {
    return AutoStartTimer().AndThen(Actions::Shoot(intake, shooter, hand)); 
}

frc2::CommandPtr ActionsAutos::ShootAndMobility(Drivetrain *drivetrain, Hand *hand, Intake *intake, Shooter *shooter, Climber *Climber) {
    return AutoStartTimer().AndThen(frc2::cmd::Sequence(
        Actions::Shoot(intake, shooter, hand),
        frc2::cmd::Wait(1_s), // wait 1 sec to make sure the shooter's finished
        frc2::cmd::Run(
            [drivetrain] {
                drivetrain->CurvatureDrive(0.3, 0); // default dt 10 percent forward
            }, {drivetrain})
        .WithTimeout(10_s)
        .AndThen([drivetrain] {drivetrain->CurvatureDrive(0, 0);})
    ));
}

frc2::CommandPtr ActionsAutos::TwoNoteMobility(Drivetrain *drivetrain, Hand *hand, Intake *intake, Shooter *shooter, Climber *climber) {
    return AutoStartTimer().AndThen(frc2::cmd::Sequence(
        Actions::Shoot(intake, shooter, hand), 
        frc2::cmd::Wait(0.5_s), // wait half a  sec to make sure the shooter's finished
        frc2::cmd::Run(
            [drivetrain] {
                drivetrain->CurvatureDrive(0.1, 0); // default dt 10 percent forward
            }, {drivetrain}).RaceWith(Actions::IntakeNote(intake)),
        frc2::cmd::Run(
            [drivetrain] {
                drivetrain->CurvatureDrive(-0.1, 0); // default dt 10 percent forward
            }, {drivetrain}).WithTimeout(3_s),
        frc2::cmd::RunOnce(
            [drivetrain] {
                drivetrain->CurvatureDrive(0.0, 0); // default dt 10 percent forward
            }, {drivetrain}),
        Actions::Shoot(intake, shooter,hand)
    ));
}