package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Field;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

//! Have to find out all the velocities, accelerations for this stuff
public class Routines {

    private Routines() {}
   
    public static Command groundIntake(ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeSubsystem) {
        return armSubsystem.setAngleCommand(Arm.intakeAngle, 0, 0, 0)
        .andThen(
            intakeSubsystem.intakeCommand(),
            stowCommand(armSubsystem)
        ).withName("Intake Ground");
    }

    public static Command scoreAmp(ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeSubsystem) {
        return armSubsystem.setAngleCommand(Arm.ampAngle, 0, 0, 0) 
        .andThen(
            intakeSubsystem.setIntakeSpeed(Intake.ampShootPower),
            Commands.waitSeconds(Intake.ampTimeToShoot.in(Seconds)),
            stowCommand(armSubsystem)
        )    
        .withName("Score Amp");
    }

    public static Command scoreSpeakerBase(ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem) {
        return armSubsystem.setAngleCommand(Arm.speakerAngle, 0, 0, 0)
        .alongWith(shooterSubsystem.spinUpFlywheelCommand(Shooter.speakerShootPower))
        .andThen(
            shooterSubsystem.releaseNoteCommand(),
            stowCommand(armSubsystem)
        )    
        .withName("Score Speaker Base");
    }

    public static Command scoreSpeakerVision(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem) {
        return armSubsystem.setAngleCommand(() -> getAngle(driveSubsystem, armSubsystem), 0, 0, 0)
        .alongWith(shooterSubsystem.spinUpFlywheelCommand(Shooter.speakerShootPower))
        .andThen(
            shooterSubsystem.releaseNoteCommand(),
            stowCommand(armSubsystem)
        )    
        .withName("Score Speaker Vision");
    }

    public static Command turnToSpeaker(DriveSubsystem driveSubsystem) {
        return driveSubsystem.turnCommand(
            () -> {
                Translation2d speakerPos = (Robot.onRedAlliance()) ? Field.redSpeakerPosition : Field.blueSpeakerPosition;
                return Radians.of(driveSubsystem.getFieldPose().getTranslation().minus(speakerPos).getAngle().getRadians());
            }
        )
        .withName("Turn to Speaker");
    }

    public static Command stowCommand(ArmSubsystem armSubsystem) {
        return armSubsystem.setAngleCommand(Arm.stowAngle, 0, 0, 0).withName("Stow");
    }

    private static Measure<Angle> getAngle(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {
        Translation2d speakerPos = (Robot.onRedAlliance()) ? Field.redSpeakerPosition : Field.blueSpeakerPosition;
        double distance = driveSubsystem.getFieldPose().getTranslation().getDistance(speakerPos);
        return armSubsystem.getTargetAngle(distance);
    }

}
