package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
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
import frc.robot.Constants.Wrist;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class Routines {

    private Routines() {}
   
    public static Command enterStow(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem) {
        return armSubsystem.setAngleCommand(Degrees.of(0)) //! Update Degree
        .alongWith(
            Commands.waitUntil(() -> armSubsystem.getAngle().gte(Degrees.of(0))) //! Update Degree
            .andThen(wristSubsystem.stowCommand())
        )
        .alongWith(
            Commands.waitUntil(() -> wristSubsystem.getAngle().gte(Wrist.stowAngle))
            .andThen(armSubsystem.stowCommand()).asProxy()
        );
    }

    public static Command leaveStow(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem) {
        return armSubsystem.setAngleCommand(Degrees.of(0)) //! Update degree
        .alongWith(
            Commands.waitUntil(() -> armSubsystem.getAngle().gte(Degrees.of(0))) //! Update degree
            .andThen(wristSubsystem.setAngleCommand(Wrist.intakeShootAngle))
        )
        .alongWith(
            Commands.waitUntil(() -> wristSubsystem.getAngle().gte(Wrist.intakeShootAngle))
            .andThen(armSubsystem.setAngleCommand(Degrees.of(0))) //! Update degree
        ); 
    }

    public static Command groundIntake(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem,IntakeShooterSubsystem intakeSubsystem) {
        return leaveStow(armSubsystem, wristSubsystem) 
        .andThen(armSubsystem.setAngleCommand(Arm.intakeAngle))
        .andThen(
            armSubsystem.runOnce(armSubsystem::stop),
            intakeSubsystem.intakeCommand()
            .alongWith(Commands.waitUntil(intakeSubsystem::isSensorTripped)
            .andThen(enterStow(armSubsystem, wristSubsystem)))
        ).withName("Intake Ground");
    }

    public static Command scoreAmp(DriveSubsystem driveSubsystem, WristSubsystem wristSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeSubsystem) {
        return leaveStow(armSubsystem, wristSubsystem) 
        .andThen(armSubsystem.setAngleCommand(Arm.ampAngle))
        .andThen(driveSubsystem.trajectoryToPoseCommand(() -> Robot.onRedAlliance() ? Field.redAmpScorePos : Field.blueAmpScorePos, ArrayList::new, false))
        .andThen(intakeSubsystem.shootAmpCommand())
        .andThen(driveSubsystem.driveDistanceVelCommand(Feet.of(1), FeetPerSecond.of(-2)))
        .andThen(enterStow(armSubsystem, wristSubsystem))
        .withName("Score Amp");
    }

    public static Command scoreSpeakerBase(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, IntakeShooterSubsystem shooterSubsystem) {
        return leaveStow(armSubsystem, wristSubsystem)
        .andThen(armSubsystem.setAngleCommand(Arm.speakerAngle))
        .alongWith(shooterSubsystem.spinUpFlywheelCommand())
        .andThen(
            shooterSubsystem.releaseNoteCommand(),
            enterStow(armSubsystem, wristSubsystem) //? maybe move out of andThen()
        )    
        .withName("Score Speaker Base");
    }

    public static Command scoreSpeaker(Measure<Angle> angle, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, IntakeShooterSubsystem shooterSubsystem) {
        return leaveStow(armSubsystem, wristSubsystem)
        .andThen(armSubsystem.setAngleCommand(angle))
        .alongWith(shooterSubsystem.spinUpFlywheelCommand())
        .andThen(
            shooterSubsystem.releaseNoteCommand(),
            enterStow(armSubsystem, wristSubsystem) //? maybe move out of andThen()
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

    public static Command turnAndScoreSpeaker(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, IntakeShooterSubsystem shooterSubsystem) {
        return turnToSpeaker(driveSubsystem)
        .alongWith(leaveStow(armSubsystem, wristSubsystem))
        .andThen(
            armSubsystem.setAngleCommand(() -> getPredictedShootAngle(driveSubsystem)),
            shooterSubsystem.spinUpFlywheelCommand()
        )
        .andThen(
            Commands.waitSeconds(0.5), // Wait for arm to settle
            shooterSubsystem.releaseNoteCommand(),
            enterStow(armSubsystem, wristSubsystem) //? maybe move out of andThen()
        )
        .withName("Aim and Score Speaker");
    }

    public static Command extendClimber(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, ClimbSubsystem climbSubsystem) {
        return enterStow(armSubsystem, wristSubsystem).andThen(climbSubsystem.setVoltage(Climb.climbPower));
    }

    public static Command retractClimber(ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, ClimbSubsystem climbSubsystem) {
        return enterStow(armSubsystem, wristSubsystem).andThen(climbSubsystem.setVoltage(Climb.climbPower.negate()));
    }

    public static Command extendAndCenterOnChain(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, WristSubsystem wristSubsystem, ClimbSubsystem climbSubsystem) {
        return extendClimber(armSubsystem, wristSubsystem, climbSubsystem)
        .alongWith(
            Commands.waitSeconds(2)
            .andThen(driveSubsystem.trajectoryToPoseCommand(() -> Field.getNearestChain(driveSubsystem.getFieldPose()), ArrayList::new, true)
        ));
    }

    public static Measure<Angle> getPredictedShootAngle(DriveSubsystem driveSubsystem) {
        if (!driveSubsystem.hasInitalizedFieldPose()) return Arm.speakerAngle;
        return Shooter.getPredictedAngle(driveSubsystem.getDistanceToSpeaker());
    }

}
