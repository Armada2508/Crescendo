package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static frc.robot.commands.Routines.groundIntake;
import static frc.robot.commands.Routines.scoreSpeakerBase;
import static frc.robot.commands.Routines.stowCommand;
import static frc.robot.commands.Routines.turnAndScoreSpeaker;
import static frc.robot.commands.Routines.turnToSpeaker;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Field;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class Autos { //! There's a lot of magic numbers in these Autos, that's just the nature of this class I think

    private Autos() {}

    /**
     * Leaves the ROBOT STARTING ZONE.
     */
    public static Command leaveStartingZone(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {
        return stowCommand(armSubsystem) 
        .andThen(driveSubsystem.driveDistanceVelCommand(Feet.of(5), FeetPerSecond.of(2.5)));
    }

    /**
     * Scores a preloaded NOTE at the SUBWOOFER and leaves the ROBOT STARTING ZONE.
     */
    public static Command scoreSpeakerBaseAndLeave(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return stowCommand(armSubsystem)
        .andThen(Commands.waitSeconds(1))
        .andThen(scoreSpeakerBase(armSubsystem, intakeShooterSubsystem))
        .andThen(driveSubsystem.driveDistanceVelCommand(Feet.of(5), FeetPerSecond.of(2.5)));
    }
    
    /**
     * Scores a preloaded NOTE into the SPEAKER using vision, faces the ALLIANCE WALL, and leaves the ROBOT STARTING ZONE.
     */
    public static Command scoreSpeakerAndLeave(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return stowCommand(armSubsystem)
        .andThen(Commands.waitSeconds(1))
        .andThen(turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem))
        .andThen(faceShooterTowardsWall(driveSubsystem))
        .andThen(driveSubsystem.driveDistanceVelCommand(Feet.of(5), FeetPerSecond.of(2.5))); 
    }

    private static Measure<Angle> robotStartingAngle;

    /**
     * Scores a preloaded NOTE into the SPEAKER using vision, faces the ALLIANCE WALL, intakes the NOTE behind it and scores it into the SPEAKER.
     */
    private static Command scoreSpeakerTwiceBehind(boolean driveBack, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        Command backUp = driveBack ? 
            driveSubsystem.setVelocityCommand(FeetPerSecond.of(3.25), FeetPerSecond.of(3.25))
            .andThen(Commands.waitUntil(() -> driveSubsystem.getDistanceToSpeaker().gte(Inches.of(70))).finallyDo(driveSubsystem::stop))
            : Commands.none();
        return stowCommand(armSubsystem)
        .andThen(Commands.waitSeconds(1)) // Let arm settle from breaking dual lock
        .andThen(Commands.either(
            Commands.runOnce(() -> robotStartingAngle = driveSubsystem.getFieldAngle())
            .andThen(
                backUp, // Back up to score
                turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem),
                faceStartingAngle(driveSubsystem),
                driveSubsystem.setVelocityCommand(FeetPerSecond.of(6), FeetPerSecond.of(6)), // Drive back to get second note
                intakeShooterSubsystem.brakeShooter(),
                groundIntake(armSubsystem, intakeShooterSubsystem)
                    .alongWith(Commands.waitUntil(intakeShooterSubsystem::isSensorTripped).finallyDo(driveSubsystem::stop))
            )
            .andThen(
                driveSubsystem.setVelocityCommand(FeetPerSecond.of(-4), FeetPerSecond.of(-4)), // Drive forward to score second note
                Commands.waitUntil(() -> driveSubsystem.getDistanceToSpeaker().lte(Inches.of(89))).finallyDo(driveSubsystem::stop),
                Commands.waitSeconds(0.25), // Pause for drivetrain to settle
                turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem) // Shoot second note
            ), 
        Commands.none(), driveSubsystem::hasInitalizedFieldPose));
    }

    public static Command scoreSpeakerTwiceBase(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return scoreSpeakerTwiceBehind(false, driveSubsystem, armSubsystem, intakeShooterSubsystem);
    }

    public static Command scoreSpeakerTwiceSide(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return scoreSpeakerTwiceBehind(true, driveSubsystem, armSubsystem, intakeShooterSubsystem);
    }

    public static Command scoreSpeakerThrice(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return scoreSpeakerTwiceBase(driveSubsystem, armSubsystem, intakeShooterSubsystem)
        .andThen( // Wait for flywheels
            faceNote(Field.blueTopNotePickupPos.getTranslation(), driveSubsystem),
            driveSubsystem.setVelocityCommand(FeetPerSecond.of(5.25), FeetPerSecond.of(5.25)), // Drive back to pick up third note
            intakeShooterSubsystem.brakeShooter(),
            groundIntake(armSubsystem, intakeShooterSubsystem)
                .alongWith(Commands.waitUntil(intakeShooterSubsystem::isSensorTripped).finallyDo(driveSubsystem::stop))
        )
        .andThen(
                turnToSpeaker(driveSubsystem),
                driveSubsystem.setVelocityCommand(FeetPerSecond.of(-3.75), FeetPerSecond.of(-3.75)), // Drive forward to score third note
                Commands.waitUntil(() -> driveSubsystem.getDistanceToSpeaker().lte(Inches.of(89))).finallyDo(driveSubsystem::stop),
                Commands.waitSeconds(0.25), // Pause for drivetrain to settle
                turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem) // Shoot third note
        );
    }

    private static Command faceShooterTowardsWall(DriveSubsystem driveSubsystem) {
        return driveSubsystem.turnCommand(
            () -> {
                if (!driveSubsystem.hasInitalizedFieldPose()) driveSubsystem.getFieldAngle();
                return (Robot.onRedAlliance()) ? Degrees.of(180) : Degrees.of(0);
            }
        );
    }

    private static Command faceStartingAngle(DriveSubsystem driveSubsystem) {
        return driveSubsystem.turnCommand(
            () -> {
                if (!driveSubsystem.hasInitalizedFieldPose()) driveSubsystem.getFieldAngle();
                return robotStartingAngle;
            }
        );
    }

    private static Command faceNote(Translation2d note, DriveSubsystem driveSubsystem) {
        return driveSubsystem.turnCommand(() ->
            Degrees.of(driveSubsystem.getFieldPose().getTranslation().minus(note).getAngle().minus(Rotation2d.fromDegrees(180)).getDegrees())
        );
    }

}
