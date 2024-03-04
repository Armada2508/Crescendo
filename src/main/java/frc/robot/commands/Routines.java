package frc.robot.commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Field;
import frc.robot.Constants.Shooter;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class Routines {

    private Routines() {}
   
    public static Command groundIntake(ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeSubsystem) {
        return armSubsystem.setAngleCommand(Arm.intakeAngle)
        .andThen(
            armSubsystem.runOnce(armSubsystem::stop),
            intakeSubsystem.intakeCommand()
            .alongWith(Commands.waitUntil(intakeSubsystem::isSensorTripped).andThen(armSubsystem.stowCommand()))
        ).withName("Intake Ground");
    }

    public static Command scoreAmp(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeSubsystem) {
        return armSubsystem.setAngleCommand(Arm.ampAngle)
        .andThen(driveSubsystem.trajectoryToPoseCommand(() -> Robot.onRedAlliance() ? Field.redAmpScorePos : Field.blueAmpScorePos, false))
        .andThen(intakeSubsystem.shootAmpCommand())
        .andThen(driveSubsystem.driveDistanceVelCommand(Feet.of(1), FeetPerSecond.of(-2)))
        .andThen(armSubsystem.stowCommand())
        .withName("Score Amp");
    }

    public static Command scoreSpeakerBase(ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem) {
        return armSubsystem.setAngleCommand(Arm.speakerAngle)
        .alongWith(shooterSubsystem.spinUpFlywheelCommand())
        .andThen(
            shooterSubsystem.releaseNoteCommand(),
            armSubsystem.stowCommand()
        )    
        .withName("Score Speaker Base");
    }

    public static Command scoreSpeakerVision(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem) {
        return armSubsystem.setAngleCommand(() -> getPredictedShootAngle(driveSubsystem))
        .alongWith(shooterSubsystem.spinUpFlywheelCommand())
        .andThen(
            shooterSubsystem.releaseNoteCommand(),
            armSubsystem.stowCommand()
        )    
        .withName("Score Speaker Vision");
    }

    public static Command turnToSpeaker(DriveSubsystem driveSubsystem) {
        return driveSubsystem.turnCommand(
            () -> {
                if (!driveSubsystem.hasInitalizedFieldPose()) return driveSubsystem.getFieldAngle();
                Translation2d speakerPos = (Robot.onRedAlliance()) ? Field.redSpeakerPosition : Field.blueSpeakerPosition;
                return Radians.of(driveSubsystem.getFieldPose().getTranslation().minus(speakerPos).getAngle().getRadians());
            }
        )
        .withName("Turn to Speaker");
    }

    public static Command turnAndScoreSpeaker(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem) {
        return turnToSpeaker(driveSubsystem)
        .alongWith(
            armSubsystem.setAngleCommand(() -> getPredictedShootAngle(driveSubsystem)),
            shooterSubsystem.spinUpFlywheelCommand()
        )
        .andThen(
            shooterSubsystem.releaseNoteCommand(),
            armSubsystem.stowCommand()
        )
        .withName("Aim and Score Speaker");
    }

    public static Measure<Angle> getPredictedShootAngle(DriveSubsystem driveSubsystem) {
        if (!driveSubsystem.hasInitalizedFieldPose()) return Arm.speakerAngle;
        return Shooter.getPredictedAngle(driveSubsystem.getDistanceToSpeaker());
    }

}
