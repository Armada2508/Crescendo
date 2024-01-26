package frc.robot.commands;

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
    
    public static Command autoSimple(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return turnToSpeaker(driveSubsystem)
        .andThen(scoreSpeakerVision(driveSubsystem, armSubsystem, intakeShooterSubsystem))
        .andThen(driveSubsystem.turnCommand(0)) //? use vision for angle?
        .andThen(driveSubsystem.driveDistanceCommand(0, 0, 0)); //? use vision for distance?, tune velocity and acceleration
    }

    public static Command autoComplex(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return turnToSpeaker(driveSubsystem)
        .andThen(scoreSpeakerVision(driveSubsystem, armSubsystem, intakeShooterSubsystem))
        //add turn command to turn to note if needed
        .andThen(driveSubsystem.trajectoryToPoseCommand(translationToTrajectoryPose(Field.mediumNoteCordinate, 0))) //assume we go for medium note, tune angle
        .andThen(groundIntake(armSubsystem, intakeShooterSubsystem))
        //add turn command to turn to speaker if needed
        .andThen(driveSubsystem.trajectoryToPoseCommand(translationToTrajectoryPose(Field.speakerPos, 0))) //tune angle
        .andThen(scoreSpeakerVision(driveSubsystem, armSubsystem, intakeShooterSubsystem))
        .andThen(driveSubsystem.driveDistanceCommand(0, 0, 0)); //tune velocity and acceleration
    }
}
