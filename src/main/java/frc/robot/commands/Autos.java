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
import frc.robot.Constants.Shooter;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class Autos {

    private Autos() {}

    private static Measure<Angle> robotStartingAngle;

    /**
     * Leaves the ROBOT STARTING ZONE.
     */
    public static Command leaveStartingZone(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {
        return driveSubsystem.driveDistanceVelCommand(Feet.of(14), FeetPerSecond.of(2.5));
    }

    /**
     * Scores a preloaded NOTE into the SPEAKER using vision (falls back to base if no tags seen).
     */
    public static Command scoreSpeaker(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem, pneumaticsSubsystem);
    }
    
    /**
     * Scores a preloaded NOTE into the SPEAKER using vision (falls back to base if no tags seen), faces the ALLIANCE WALL, and leaves the ROBOT STARTING ZONE.
     */
    public static Command scoreSpeakerAndLeave(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return scoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem, pneumaticsSubsystem)
        .andThen(
            faceShooterTowardsWall(driveSubsystem),
            driveSubsystem.driveDistanceVelCommand(Feet.of(14), FeetPerSecond.of(8))
        );
    }

    /**
     * Scores a preloaded NOTE at the SUBWOOFER into the SPEAKER, backs up to get a second NOTE and scores it in the SPEAKER.
     */
    public static Command scoreSpeakerTwiceBase(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem, ClimbSubsystem climbSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return scoreSpeakerBase(armSubsystem, intakeShooterSubsystem, pneumaticsSubsystem, false)
        .andThen(
            Commands.waitSeconds(1),
            intakeShooterSubsystem.brakeShooter(),
            driveSubsystem.motionMagicVelocityCommand(FeetPerSecond.of(6.5), FeetPerSecond.of(6.5), FeetPerSecondSquared.of(12)) // Drive back to get second note
                .alongWith(
                    armSubsystem.setAngleCommand(Arm.intakeAngle),
                    intakeShooterSubsystem.intakeCommand(),
                    Commands.waitUntil(intakeShooterSubsystem::isSensorTripped).finallyDo(driveSubsystem::stop).withTimeout(1.25)
                ),
            armSubsystem.stowCommand()
                .alongWith( // Drive forward to score second note
                    driveSubsystem.setVelocityCommand(FeetPerSecond.of(-5), FeetPerSecond.of(-5))
                    .andThen(Commands.waitUntil(() -> Field.getDistanceToSpeaker(driveSubsystem.getFieldPose()).lte(Inches.of(66))).finallyDo(driveSubsystem::stop).withTimeout(3))
                ), 
            Commands.waitSeconds(0.25), // Pause for drivetrain to settle
            turnToSpeaker(driveSubsystem),
            armSubsystem.setAngleCommand(Degrees.of(44)) // Hardcode angle
                .alongWith(intakeShooterSubsystem.spinUpFlywheelCommand()),
            intakeShooterSubsystem.releaseNoteCommand()   
        );
    }

    /**
     * The third NOTE is the one positioned to the right (in reference to the blue alliance) of the SPEAKER. 
     * Scores a preloaded NOTE at the SUBWOOFER into the SPEAKER, backs up to get a second NOTE, scores it in the SPEAKER and then grabs 
     * a third NOTE and scores it in the SPEAKER.
     */
    public static Command scoreSpeakerThrice(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem, ClimbSubsystem climbSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return scoreSpeakerTwiceBase(driveSubsystem, armSubsystem, intakeShooterSubsystem, climbSubsystem, pneumaticsSubsystem)
        .andThen( 
            faceNote(Note.TOP, driveSubsystem).withTimeout(1.5),
            Commands.waitSeconds(0.25),
            driveSubsystem.motionMagicVelocityCommand(FeetPerSecond.of(6), FeetPerSecond.of(6), FeetPerSecondSquared.of(10)), // Drive back to pick up third note
            armSubsystem.setAngleCommand(Arm.intakeAngle),
            intakeShooterSubsystem.brakeShooter(),
            intakeShooterSubsystem.intakeCommand()
                .alongWith(
                    Commands.waitUntil(intakeShooterSubsystem::isSensorTripped).finallyDo(driveSubsystem::stop).withTimeout(1),
                    Commands.waitUntil(intakeShooterSubsystem::isSensorTripped).andThen(armSubsystem.stowCommand())
                ),
            turnToSpeaker(driveSubsystem),
            driveSubsystem.setVelocityCommand(FeetPerSecond.of(-6), FeetPerSecond.of(-6)), // Drive forward to score third note
            Commands.waitUntil(() -> Field.getDistanceToSpeaker(driveSubsystem.getFieldPose()).lte(Inches.of(78))).finallyDo(driveSubsystem::stop).withTimeout(3),
            Commands.waitSeconds(0.25), // Pause for drivetrain to settle
            turnToSpeaker(driveSubsystem)
                .alongWith(
                    intakeShooterSubsystem.spinUpFlywheelCommand(),
                    armSubsystem.setAngleCommand(() -> {
                        if (!driveSubsystem.hasInitalizedFieldPose()) return Arm.speakerBaseAngle;
                        return Shooter.getPredictedAngle(Field.getDistanceToSpeaker(driveSubsystem.getFieldPose()));
                    })
                ),
            Commands.waitSeconds(0.5), // Wait for arm to settle
            intakeShooterSubsystem.releaseNoteCommand()
        );
    }

    /**
     * This is for when positioned to the right (in reference to the blue alliance) of the SPEAKER.
     * Scores a preloaded NOTE at the SUBWOOFER into the SPEAKER, backs up to get a second NOTE and scores it in the SPEAKER.
     */
    @Deprecated
    public static Command scoreSpeakerTwiceSide(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return armSubsystem.stowCommand()
        .andThen(
            Commands.runOnce(() -> robotStartingAngle = driveSubsystem.getFieldAngle()),
            driveSubsystem.motionMagicVelocityCommand(FeetPerSecond.of(3.25), FeetPerSecond.of(3.25), FeetPerSecondSquared.of(3.25)),
            Commands.waitUntil(() -> Field.getDistanceToSpeaker(driveSubsystem.getFieldPose()).gte(Inches.of(70))).finallyDo(driveSubsystem::stop).withTimeout(4), // Back up to score
            turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem, pneumaticsSubsystem),
            faceStartingAngle(driveSubsystem), //? Could just use faceNote() here
            driveSubsystem.motionMagicVelocityCommand(FeetPerSecond.of(6), FeetPerSecond.of(6), FeetPerSecondSquared.of(6)), // Drive back to get second note
            groundIntake(armSubsystem, intakeShooterSubsystem, pneumaticsSubsystem)
                .alongWith(Commands.waitUntil(intakeShooterSubsystem::isSensorTripped).finallyDo(driveSubsystem::stop).withTimeout(1.5)),
            driveSubsystem.motionMagicVelocityCommand(FeetPerSecond.of(-4), FeetPerSecond.of(-4), FeetPerSecondSquared.of(4)), // Drive forward to score second note
            Commands.waitUntil(() -> Field.getDistanceToSpeaker(driveSubsystem.getFieldPose()).lte(Inches.of(89))).finallyDo(driveSubsystem::stop).withTimeout(4),
            Commands.waitSeconds(0.25), // Pause for drivetrain to settle
            turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem, pneumaticsSubsystem) // Shoot second note
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
