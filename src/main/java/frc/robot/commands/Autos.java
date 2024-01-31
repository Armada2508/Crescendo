package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.commands.Routines.groundIntake;
import static frc.robot.commands.Routines.scoreSpeakerVision;
import static frc.robot.commands.Routines.translationToTrajectoryPose;
import static frc.robot.commands.Routines.turnToSpeaker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Field;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class Autos {

    private Autos() {}
    //! Have to tune accelerations and velocitys for everything
    public static Command autoSimple(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return turnToSpeaker(driveSubsystem)
        .andThen(scoreSpeakerVision(driveSubsystem, armSubsystem, intakeShooterSubsystem))
        .andThen(driveSubsystem.turnCommand(Degrees.of(180))) //turn to leave //? use vision for angle?
        .andThen(driveSubsystem.driveDistanceCommand(Meters.of(1.5), 0, 0)); //drive out //? adjust distance to leave at week zero
    }

    public static Command autoComplex(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return turnToSpeaker(driveSubsystem)
        .andThen(scoreSpeakerVision(driveSubsystem, armSubsystem, intakeShooterSubsystem))
        .andThen(driveSubsystem.trajectoryToPoseCommand(Field.lowerNotePickupLocation)) //assume we go for medium note
        .andThen(groundIntake(armSubsystem, intakeShooterSubsystem))
        .andThen(driveSubsystem.trajectoryToPoseCommand(translationToTrajectoryPose(Field.speakerPos, 0))) //tune angle
        .andThen(scoreSpeakerVision(driveSubsystem, armSubsystem, intakeShooterSubsystem));
    }
}
