package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.commands.Routines.groundIntake;
import static frc.robot.commands.Routines.scoreSpeakerVision;
import static frc.robot.commands.Routines.turnToSpeaker;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Field;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class Autos {

    private Autos() {}

    public static Command leaveStartingZone(DriveSubsystem driveSubsystem) {
        return driveSubsystem.driveDistanceCommand(Feet.of(5), 0, 0);
    }

    //! Have to tune accelerations and velocities for everything
    public static Command autoSimple(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return turnToSpeaker(driveSubsystem)
        .andThen(scoreSpeakerVision(driveSubsystem, armSubsystem, intakeShooterSubsystem))
        .andThen(driveSubsystem.turnCommand(
            () -> (Robot.onRedAlliance()) ? Degrees.of(0) : Degrees.of(180) // Turn to face alliance wall
        ))
        .andThen(driveSubsystem.driveDistanceCommand(Meters.of(-1.5), 0, 0)); //? adjust distance to leave at week zero
    }

    // Lot of on the fly trajectory generation might be poor
    public static Command autoComplex(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return turnToSpeaker(driveSubsystem)
        .andThen(scoreSpeakerVision(driveSubsystem, armSubsystem, intakeShooterSubsystem))
        .andThen(driveSubsystem.trajectoryToPoseCommand(
            () -> (Robot.onRedAlliance()) ? getRedPosition(Field.lowerNotePickupLocation) : Field.lowerNotePickupLocation
        )) 
        .andThen(groundIntake(armSubsystem, intakeShooterSubsystem))
        .andThen(driveSubsystem.trajectoryToPoseCommand(
            () -> (Robot.onRedAlliance()) ? Field.speakerPosRed : Field.speakerPosBlue
        )) 
        .andThen(scoreSpeakerVision(driveSubsystem, armSubsystem, intakeShooterSubsystem));
    }

    private static Pose2d getRedPosition(Pose2d pose) {
        return new Pose2d(Field.fieldLength.in(Meters) - pose.getX(), pose.getY(), pose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

}
