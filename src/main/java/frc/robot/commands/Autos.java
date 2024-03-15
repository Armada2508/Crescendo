package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
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

public class Autos {

    private Autos() {}

    private static Measure<Angle> robotStartingAngle;

    /**
     * Leaves the ROBOT STARTING ZONE.
     */
    public static Command leaveStartingZone(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {
        return armSubsystem.stowCommand()
        .andThen(
            armSubsystem.initArmAngle(),
            driveSubsystem.driveDistanceVelCommand(Feet.of(5), FeetPerSecond.of(2.5))
        );
    }

    /**
     * Scores a preloaded NOTE into the SPEAKER using vision (falls back to base if no tags seen).
     */
    public static Command scoreSpeaker(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return armSubsystem.stowCommand()
        .andThen(
            armSubsystem.initArmAngle(),
            turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem)
        );
    }
    
    /**
     * Scores a preloaded NOTE into the SPEAKER using vision (falls back to base if no tags seen), faces the ALLIANCE WALL, and leaves the ROBOT STARTING ZONE.
     */
    public static Command scoreSpeakerAndLeave(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return scoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem)
        .andThen(
            faceShooterTowardsWall(driveSubsystem),
            driveSubsystem.driveDistanceVelCommand(Feet.of(14), FeetPerSecond.of(2.5))
        );
    }

    /**
     * Scores a preloaded NOTE at the SUBWOOFER into the SPEAKER, backs up to get a second NOTE and scores it in the SPEAKER.
     */
    public static Command scoreSpeakerTwiceBase(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem, ClimbSubsystem climbSubsystem) {
        return armSubsystem.stowCommand()
        .andThen(
            scoreSpeakerBase(armSubsystem, intakeShooterSubsystem),
            Commands.waitSeconds(1),
            intakeShooterSubsystem.brakeShooter(),
            driveSubsystem.motionMagicVelocityCommand(FeetPerSecond.of(7.5), FeetPerSecond.of(7.5), FeetPerSecondSquared.of(12)) // Drive back to get second note
                .alongWith(
                    armSubsystem.initArmAngle(),
                    intakeShooterSubsystem.intakeCommand(),
                    Commands.waitUntil(intakeShooterSubsystem::isSensorTripped).finallyDo(driveSubsystem::stop).withTimeout(1.25)
                ),
            armSubsystem.stowCommand()
                .alongWith( // Drive forward to score second note
                    driveSubsystem.setVelocityCommand(FeetPerSecond.of(-4), FeetPerSecond.of(-4))
                    .andThen(Commands.waitUntil(() -> Field.getDistanceToSpeaker(driveSubsystem.getFieldPose()).lte(Inches.of(60))).finallyDo(driveSubsystem::stop).withTimeout(3))
                ), 
            Commands.waitSeconds(0.25), // Pause for drivetrain to settle
            turnToSpeaker(driveSubsystem),
            armSubsystem.setAngleCommand(Degrees.of(41.5)) // Manually adjust angle for distance above
                .alongWith(intakeShooterSubsystem.spinUpFlywheelCommand()),
            intakeShooterSubsystem.releaseNoteCommand()   
        );
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
            intakeShooterSubsystem.brakeShooter(),
            intakeShooterSubsystem.intakeCommand()
                .alongWith(Commands.waitUntil(intakeShooterSubsystem::isSensorTripped).finallyDo(driveSubsystem::stop).withTimeout(1)),
            armSubsystem.stowCommand(),
            turnToSpeaker(driveSubsystem),
            driveSubsystem.setVelocityCommand(FeetPerSecond.of(-6), FeetPerSecond.of(-6)), // Drive forward to score third note
            Commands.waitUntil(() -> Field.getDistanceToSpeaker(driveSubsystem.getFieldPose()).lte(Inches.of(78))).finallyDo(driveSubsystem::stop).withTimeout(3),
            Commands.waitSeconds(0.25), // Pause for drivetrain to settle
            turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem) // Shoot third note
        );
    }

    /**
     * This is for when positioned to the right (in reference to the blue alliance) of the SPEAKER.
     * Scores a preloaded NOTE at the SUBWOOFER into the SPEAKER, backs up to get a second NOTE and scores it in the SPEAKER.
     */
    public static Command scoreSpeakerTwiceSide(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return armSubsystem.stowCommand()
        .andThen(
            Commands.runOnce(() -> robotStartingAngle = driveSubsystem.getFieldAngle()),
            driveSubsystem.motionMagicVelocityCommand(FeetPerSecond.of(3.25), FeetPerSecond.of(3.25), FeetPerSecondSquared.of(3.25)),
            Commands.waitUntil(() -> Field.getDistanceToSpeaker(driveSubsystem.getFieldPose()).gte(Inches.of(70))).finallyDo(driveSubsystem::stop).withTimeout(4), // Back up to score
            turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem),
            faceStartingAngle(driveSubsystem), //? Could just use faceNote() here
            driveSubsystem.motionMagicVelocityCommand(FeetPerSecond.of(6), FeetPerSecond.of(6), FeetPerSecondSquared.of(6)), // Drive back to get second note
            groundIntake(armSubsystem, intakeShooterSubsystem)
                .alongWith(Commands.waitUntil(intakeShooterSubsystem::isSensorTripped).finallyDo(driveSubsystem::stop).withTimeout(1.5)),
            driveSubsystem.motionMagicVelocityCommand(FeetPerSecond.of(-4), FeetPerSecond.of(-4), FeetPerSecondSquared.of(4)), // Drive forward to score second note
            Commands.waitUntil(() -> Field.getDistanceToSpeaker(driveSubsystem.getFieldPose()).lte(Inches.of(89))).finallyDo(driveSubsystem::stop).withTimeout(4),
            Commands.waitSeconds(0.25), // Pause for drivetrain to settle
            turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem) // Shoot second note
        );
    }

    /**
     * Faces the robot's shooter towards its ALLIANCE WALL.
     */
    private static Command faceShooterTowardsWall(DriveSubsystem driveSubsystem) {
        return driveSubsystem.turnCommand(() -> (Robot.onRedAlliance()) ? Degrees.of(180) : Degrees.of(0));
    }

    /**
     * Faces the robot towards the angle it was at when AUTO started.
     */
    private static Command faceStartingAngle(DriveSubsystem driveSubsystem) {
        return driveSubsystem.turnCommand(() -> robotStartingAngle);
    }

    /**
     * Faces the robot's intake towards a NOTE.
     */
    private static Command faceNote(Note note, DriveSubsystem driveSubsystem) {
        return driveSubsystem.turnCommand(() -> {
            return Degrees.of(driveSubsystem.getFieldPose().getTranslation().minus(Field.getNote(note)).getAngle().minus(Rotation2d.fromDegrees(180)).getDegrees());
        });
    }

}
