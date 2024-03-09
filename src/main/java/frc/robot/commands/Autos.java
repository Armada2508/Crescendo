package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.FeetPerSecondSquared;
import static frc.robot.commands.Routines.groundIntake;
import static frc.robot.commands.Routines.scoreSpeakerBase;
import static frc.robot.commands.Routines.turnAndScoreSpeaker;
import static frc.robot.commands.Routines.turnToSpeaker;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Field;
import frc.robot.Constants.Field.Note;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class Autos { //! There's a lot of magic numbers in these Autos, that's just the nature of this class I think

    private Autos() {}

    private static Measure<Angle> robotStartingAngle;

    /**
     * Leaves the ROBOT STARTING ZONE.
     */
    public static Command leaveStartingZone(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {
        return armSubsystem.stowCommand()
        .andThen(armSubsystem.initArmAngle())
        .andThen(driveSubsystem.driveDistanceVelCommand(Feet.of(5), FeetPerSecond.of(2.5)));
    }

    /**
     * Scores a preloaded NOTE at the SUBWOOFER and leaves the ROBOT STARTING ZONE.
     */
    public static Command scoreSpeakerBaseAndLeave(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return armSubsystem.stowCommand()
        .andThen(armSubsystem.initArmAngle())
        .andThen(scoreSpeakerBase(armSubsystem, intakeShooterSubsystem))
        .andThen(driveSubsystem.driveDistanceVelCommand(Feet.of(5), FeetPerSecond.of(2.5)));
    }
    
    /**
     * Scores a preloaded NOTE into the SPEAKER using vision, faces the ALLIANCE WALL, and leaves the ROBOT STARTING ZONE.
     */
    public static Command scoreSpeakerAndLeave(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return armSubsystem.stowCommand() 
        .andThen(armSubsystem.initArmAngle())
        .andThen(turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem))
        .andThen(faceShooterTowardsWall(driveSubsystem))
        .andThen(driveSubsystem.driveDistanceVelCommand(Feet.of(5), FeetPerSecond.of(2.5))); 
    }

    /**
     * Scores a preloaded NOTE at the SUBWOOFER into the SPEAKER, backs up to get a second NOTE and scores it in the SPEAKER.
     */
    public static Command scoreSpeakerTwiceBase(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem, ClimbSubsystem climbSubsystem) {
        return 
        (Commands.either(
            armSubsystem.stowCommand()
            .andThen(
                scoreSpeakerBase(armSubsystem, intakeShooterSubsystem),
                // Commands.waitUntil(() -> intakeShooterSubsystem.getShooterRPM() < 2000),
                Commands.waitSeconds(1),
                intakeShooterSubsystem.brakeShooter(),
                driveSubsystem.motionMagicVelocityCommand(FeetPerSecond.of(7.5), FeetPerSecond.of(7.5), FeetPerSecondSquared.of(12)) // Drive back to get second note
                    .alongWith(
                        armSubsystem.initArmAngle(),
                        // intakeShooterSubsystem.intakeFirstTryCommand(0.4, Seconds.of(0.07)),
                        intakeShooterSubsystem.intakeCommand(),
                        Commands.waitUntil(intakeShooterSubsystem::isSensorTripped).finallyDo(driveSubsystem::stop).withTimeout(1.25)
                    )
            )
            .andThen(
                armSubsystem.stowCommand()
                    .alongWith( // Drive forward to score second note
                        driveSubsystem.setVelocityCommand(FeetPerSecond.of(-4), FeetPerSecond.of(-4))
                        .andThen(Commands.waitUntil(() -> driveSubsystem.getDistanceToSpeaker().lte(Inches.of(60))).finallyDo(driveSubsystem::stop))
                    ), 
                Commands.waitSeconds(0.25), // Pause for drivetrain to settle
                turnToSpeaker(driveSubsystem),
                armSubsystem.setAngleCommand(Degrees.of(41.5))
                    .alongWith(intakeShooterSubsystem.spinUpFlywheelCommand())
                    .andThen(
                        intakeShooterSubsystem.releaseNoteCommand()
                    )     // Manually adjust angle for distance above
            ), 
        Commands.none(), driveSubsystem::hasInitalizedFieldPose));
    }

    /**
     * The third NOTE is the one positioned to the right (in reference to the blue alliance) of the SPEAKER. 
     * Scores a preloaded NOTE at the SUBWOOFER into the SPEAKER, backs up to get a second NOTE, scores it in the SPEAKER and then grabs 
     * a third NOTE and scores it in the SPEAKER.
     */
    public static Command scoreSpeakerThrice(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem, ClimbSubsystem climbSubsystem) {
        return scoreSpeakerTwiceBase(driveSubsystem, armSubsystem, intakeShooterSubsystem, climbSubsystem)
        .andThen( 
            faceNote(Note.TOP, driveSubsystem).withTimeout(1.5),
            Commands.waitSeconds(0.25),
            driveSubsystem.motionMagicVelocityCommand(FeetPerSecond.of(6), FeetPerSecond.of(6), FeetPerSecondSquared.of(10)), // Drive back to pick up third note
            armSubsystem.setAngleCommand(Arm.intakeAngle),
            intakeShooterSubsystem.intakeFirstTryCommand(0.5, Seconds.of(0.02))
                .alongWith(Commands.waitUntil(intakeShooterSubsystem::isSensorTripped).finallyDo(driveSubsystem::stop).withTimeout(1)),
            armSubsystem.stowCommand(),
            turnToSpeaker(driveSubsystem),
            driveSubsystem.setVelocityCommand(FeetPerSecond.of(-6), FeetPerSecond.of(-6)), // Drive forward to score third note
            Commands.waitUntil(() -> driveSubsystem.getDistanceToSpeaker().lte(Inches.of(78))).finallyDo(driveSubsystem::stop),
            Commands.waitSeconds(0.25), // Pause for drivetrain to settle
            turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem) // Shoot third note
        );
    }

    /**
     * This is for when positioned to the right (in reference to the blue alliance) of the SPEAKER.
     * Scores a preloaded NOTE at the SUBWOOFER into the SPEAKER, backs up to get a second NOTE and scores it in the SPEAKER.
     */
    public static Command scoreSpeakerTwiceSide(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return armSubsystem.initArmAngle()
        .andThen(Commands.either(
            Commands.runOnce(() -> robotStartingAngle = driveSubsystem.getFieldAngle())
            .andThen(
                driveSubsystem.motionMagicVelocityCommand(FeetPerSecond.of(3.25), FeetPerSecond.of(3.25), FeetPerSecondSquared.of(3.25))
                    .andThen(Commands.waitUntil(() -> driveSubsystem.getDistanceToSpeaker().gte(Inches.of(70))).finallyDo(driveSubsystem::stop)), // Back up to score
                turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem),
                faceStartingAngle(driveSubsystem),
                driveSubsystem.motionMagicVelocityCommand(FeetPerSecond.of(6), FeetPerSecond.of(6), FeetPerSecondSquared.of(6)), // Drive back to get second note
                groundIntake(armSubsystem, intakeShooterSubsystem)
                    .alongWith(Commands.waitUntil(intakeShooterSubsystem::isSensorTripped).finallyDo(driveSubsystem::stop).withTimeout(1.5))
            )
            .andThen(
                driveSubsystem.motionMagicVelocityCommand(FeetPerSecond.of(-4), FeetPerSecond.of(-4), FeetPerSecondSquared.of(4)), // Drive forward to score second note
                Commands.waitUntil(() -> driveSubsystem.getDistanceToSpeaker().lte(Inches.of(89))).finallyDo(driveSubsystem::stop),
                Commands.waitSeconds(0.25), // Pause for drivetrain to settle
                turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem) // Shoot second note
            ), 
        Commands.none(), driveSubsystem::hasInitalizedFieldPose));
    }

    /**
     * Faces the robot's shooter towards its ALLIANCE WALL.
     */
    private static Command faceShooterTowardsWall(DriveSubsystem driveSubsystem) {
        return driveSubsystem.turnCommand(() -> {
            if (!driveSubsystem.hasInitalizedFieldPose()) driveSubsystem.getFieldAngle();
            return (Robot.onRedAlliance()) ? Degrees.of(180) : Degrees.of(0);
        });
    }

    /**
     * Faces the robot towards the angle it was at when AUTO started.
     */
    private static Command faceStartingAngle(DriveSubsystem driveSubsystem) {
        return driveSubsystem.turnCommand(() -> {
            if (!driveSubsystem.hasInitalizedFieldPose()) driveSubsystem.getFieldAngle();
            return robotStartingAngle;
        });
    }

    /**
     * Faces the robot's intake towards a NOTE.
     */
    public static Command faceNote(Note note, DriveSubsystem driveSubsystem) {
        return driveSubsystem.turnCommand(() -> {
            if (!driveSubsystem.hasInitalizedFieldPose()) driveSubsystem.getFieldAngle();
            return Degrees.of(driveSubsystem.getFieldPose().getTranslation().minus(Field.getNote(note)).getAngle().minus(Rotation2d.fromDegrees(180)).getDegrees());
        });
    }

}
