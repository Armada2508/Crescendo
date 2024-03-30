package frc.robot.commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Climb;
import frc.robot.Constants.Field;
import frc.robot.Constants.Shooter;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class Routines {

    private Routines() {}

    public static Command enterStow(ArmSubsystem armSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return Commands.either(armSubsystem.setAngleCommand(Arm.retractAngle), Commands.none(), () -> armSubsystem.getAngle().lt(Arm.retractAngle) && pneumaticsSubsystem.isExtended())
        .andThen(pneumaticsSubsystem.retract())
        .andThen(armSubsystem.setAngleCommand(Arm.stowAngle))
        .withName("Enter Full Stow");
    }

    public static Command leaveStow(ArmSubsystem armSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return armSubsystem.setAngleCommand(Arm.retractAngle)
        .andThen(pneumaticsSubsystem.extend())
        .withName("Leave Full Stow");
    }
   
    public static Command groundIntake(ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return leaveStow(armSubsystem, pneumaticsSubsystem)
        .andThen(armSubsystem.setAngleCommand(Arm.intakeAngle))
        .andThen(
            armSubsystem.runOnce(armSubsystem::stop),
            intakeSubsystem.intakeCommand()
            .alongWith(Commands.waitUntil(intakeSubsystem::isSensorTripped).andThen(enterStow(armSubsystem, pneumaticsSubsystem)))
        )
        .withName("Intake Ground");
    }

    public static Command scoreAmp(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return leaveStow(armSubsystem, pneumaticsSubsystem)
        .andThen(armSubsystem.setAngleCommand(Arm.ampAngle))
        .andThen(driveSubsystem.trajectoryToPoseCommand(() -> Robot.onRedAlliance() ? Field.redAmpScorePos : Field.blueAmpScorePos, ArrayList::new, false))
        .andThen(intakeSubsystem.shootAmpCommand())
        .andThen(driveSubsystem.driveDistanceVelCommand(Feet.of(1), FeetPerSecond.of(-2)))
        .andThen(armSubsystem.stowCommand())
        .andThen(enterStow(armSubsystem, pneumaticsSubsystem))
        .withName("Score Amp");
    }

    public static Command scoreSpeakerBase(ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return leaveStow(armSubsystem, pneumaticsSubsystem)
        .andThen(armSubsystem.setAngleCommand(Arm.speakerBaseAngle))
        .alongWith(shooterSubsystem.spinUpFlywheelCommand())
        .andThen(
            shooterSubsystem.releaseNoteCommand(),
            enterStow(armSubsystem, pneumaticsSubsystem)
        )
        .withName("Score Speaker Base");
    }

    public static Command scoreSpeaker(Measure<Angle> angle, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return leaveStow(armSubsystem, pneumaticsSubsystem)
        .andThen(armSubsystem.setAngleCommand(angle))
        .alongWith(shooterSubsystem.spinUpFlywheelCommand())
        .andThen(
            shooterSubsystem.releaseNoteCommand(),
            enterStow(armSubsystem, pneumaticsSubsystem)
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

    public static Command turnAndScoreSpeaker(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return turnToSpeaker(driveSubsystem)
        .alongWith(
            leaveStow(armSubsystem, pneumaticsSubsystem),
            shooterSubsystem.spinUpFlywheelCommand()
        )
        .andThen(
            armSubsystem.setAngleCommand(() -> {
                if (!driveSubsystem.hasInitalizedFieldPose()) return Arm.speakerBaseAngle;
                return Shooter.getPredictedAngle(Field.getDistanceToSpeaker(driveSubsystem.getFieldPose()));
            }),
            Commands.waitSeconds(0.5), // Wait for arm to settle
            shooterSubsystem.releaseNoteCommand(),
            enterStow(armSubsystem, pneumaticsSubsystem)
        )
        .withName("Aim and Score Speaker");
    }

    public static Command extendClimber(ArmSubsystem armSubsystem, ClimbSubsystem climbSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return enterStow(armSubsystem, pneumaticsSubsystem).andThen(climbSubsystem.setVoltage(Climb.climbPower));
    }

    public static Command retractClimber(ArmSubsystem armSubsystem, ClimbSubsystem climbSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return enterStow(armSubsystem, pneumaticsSubsystem).andThen(climbSubsystem.setVoltage(Climb.climbPower.negate()));
    }

    public static Command extendAndCenterOnChain(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, ClimbSubsystem climbSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return extendClimber(armSubsystem, climbSubsystem, pneumaticsSubsystem)
        .alongWith(
            Commands.waitSeconds(2)
            .andThen(driveSubsystem.trajectoryToPoseCommand(() -> Field.getNearestChain(driveSubsystem.getFieldPose()), ArrayList::new, true)
        ));
    }

}
