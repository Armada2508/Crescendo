package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static frc.robot.commands.Routines.groundIntake;
import static frc.robot.commands.Routines.stowCommand;
import static frc.robot.commands.Routines.turnAndScoreSpeaker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class Autos { //! Have to tune accelerations and velocities for everything

    private Autos() {}

    // /**
    //  * Leaves the ROBOT STARTING ZONE
    //  */
    // public static Command leaveStartingZone(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {
    //     return stowCommand(armSubsystem) 
    //     .andThen(driveSubsystem.driveDistanceVelCommand(Feet.of(5), FeetPerSecond.of(2.5)));
    // }

    // /**
    //  * Scores a preloaded NOTE at the SUBWOOFER and leaves the ROBOT STARTING ZONE
    //  * @param driveSubsystem
    //  * @param armSubsystem
    //  * @param intakeShooterSubsystem
    //  * @return
    //  */
    // public static Command scoreSpeakerBaseAndLeave(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
    //     return stowCommand(armSubsystem)
    //     .andThen(scoreSpeakerBase(armSubsystem, intakeShooterSubsystem))
    //     .andThen(driveSubsystem.driveDistanceVelCommand(Feet.of(4), FeetPerSecond.of(2)));
    // }
    
    // /**
    //  * Scores a preloaded NOTE into the SPEAKER within reasonable distance, turns, and leaves the ROBOT STARTING ZONE
    //  */
    // public static Command scoreSpeakerAndLeave(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
    //     return stowCommand(armSubsystem)
    //     .andThen(Commands.waitSeconds(1))
    //     .andThen(turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem))
    //     .andThen(faceShooterTowardsWall(driveSubsystem))
    //     .andThen(driveSubsystem.driveDistanceVelCommand(Feet.of(5), FeetPerSecond.of(2.5))); 
    // }

    public static Command scoreSpeakerTwiceBehind(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return stowCommand(armSubsystem)
        .andThen(Commands.waitSeconds(1))
        .andThen(turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem))
        .andThen(
            Commands.waitSeconds(2) // Wait for flywheels to spin down
            .alongWith(faceShooterTowardsWall(driveSubsystem))
        )
        .andThen(
            intakeShooterSubsystem.brakeShooter(),
            driveSubsystem.setVelocityCommand(FeetPerSecond.of(4), FeetPerSecond.of(4)),
            groundIntake(armSubsystem, intakeShooterSubsystem)
                .alongWith(Commands.waitUntil(intakeShooterSubsystem::isSensorTripped).andThen(driveSubsystem.runOnce(driveSubsystem::stop)))
        )
        .andThen(
            driveSubsystem.setVelocityCommand(FeetPerSecond.of(-3), FeetPerSecond.of(-3)),
            Commands.waitUntil(() -> driveSubsystem.getDistanceToSpeaker().lte(Inches.of(92))).finallyDo(driveSubsystem::stop),
            Commands.waitSeconds(1),
            turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem)
        );
    }

    // // Lot of on the fly trajectory generation might be poor
    // /**
    //  * Scores a preloaded NOTE into the SPEAKER, grabs another NOTE on the field and then scores it.
    //  */
    // public static Command scoreSpeakerTwice(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
    //     return turnToSpeaker(driveSubsystem)
    //     .andThen(scoreSpeakerVision(driveSubsystem, armSubsystem, intakeShooterSubsystem))
    //     .andThen(driveSubsystem.trajectoryToPoseCommand(
    //         () -> (Robot.onRedAlliance()) ? getRedPosition(Field.blueLowNotePickupPos) : Field.blueLowNotePickupPos, false
    //     )) 
    //     .andThen(groundIntake(armSubsystem, intakeShooterSubsystem))
    //     .andThen(driveSubsystem.trajectoryToPoseCommand(
    //         () -> (Robot.onRedAlliance()) ? Field.redSpeakerBaseScorePos : Field.blueSpeakerBaseScorePos, true
    //     )) 
    //     .andThen(scoreSpeakerVision(driveSubsystem, armSubsystem, intakeShooterSubsystem));
    // }

    // private static Pose2d getRedPosition(Pose2d pose) {
    //     return new Pose2d(Field.fieldLength.in(Meters) - pose.getX(), pose.getY(), pose.getRotation().plus(Rotation2d.fromDegrees(180)));
    // }

    private static Command faceShooterTowardsWall(DriveSubsystem driveSubsystem) {
        return driveSubsystem.turnCommand(
            () -> {
                if (!driveSubsystem.hasInitalizedFieldPose()) Degrees.of(driveSubsystem.getFieldPose().getRotation().getDegrees());
                return (Robot.onRedAlliance()) ? Degrees.of(180) : Degrees.of(0);
            }
        );
    }

}
