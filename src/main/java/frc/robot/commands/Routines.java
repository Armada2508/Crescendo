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
import frc.robot.subsystems.VisionSubsystem;

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
        .andThen(driveSubsystem.trajectoryToPoseCommand(() -> Robot.onRedAlliance() ? Field.redAmpScorePos : Field.blueAmpScorePos, ArrayList::new, false))
        .andThen(intakeSubsystem.shootAmpCommand())
        .andThen(driveSubsystem.driveDistanceVelCommand(Feet.of(1), FeetPerSecond.of(-2)))
        .andThen(armSubsystem.stowCommand())
        .withName("Score Amp");
    }

    public static Command scoreSpeakerBase(ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem) {
        return armSubsystem.setAngleCommand(Arm.speakerBaseAngle)
        .alongWith(shooterSubsystem.spinUpFlywheelCommand())
        .andThen(
            shooterSubsystem.releaseNoteCommand(),
            armSubsystem.stowCommand()
        )    
        .withName("Score Speaker Base");
    }

    public static Command scoreSpeaker(Measure<Angle> angle, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem) {
        return armSubsystem.setAngleCommand(angle)
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
                Translation2d speakerPos = (Robot.onRedAlliance()) ? Field.redSpeakerPosition : Field.blueSpeakerPosition;
                return Radians.of(driveSubsystem.getFieldPose().getTranslation().minus(speakerPos).getAngle().getRadians());
            }
        )
        .withName("Turn to Speaker");
    }

    public static Command turnAndScoreSpeaker(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem) {
        return turnToSpeaker(driveSubsystem)
        .alongWith(
            armSubsystem.setAngleCommand(() -> {
                if (!driveSubsystem.hasInitalizedFieldPose()) return Arm.speakerBaseAngle;
                return Shooter.getPredictedAngle(Field.getDistanceToSpeaker(driveSubsystem.getFieldPose()));
            }),
            shooterSubsystem.spinUpFlywheelCommand()
        )
        .andThen(
            Commands.waitSeconds(0.5), // Wait for arm to settle
            shooterSubsystem.releaseNoteCommand(),
            armSubsystem.stowCommand()
        )
        .withName("Aim and Score Speaker");
    }

    public static Command extendClimber(ArmSubsystem armSubsystem, ClimbSubsystem climbSubsystem) {
        return armSubsystem.stowCommand().andThen(climbSubsystem.setVoltage(Climb.climbPower));
    }

    public static Command retractClimber(ArmSubsystem armSubsystem, ClimbSubsystem climbSubsystem) {
        return armSubsystem.stowCommand().andThen(climbSubsystem.setVoltage(Climb.climbPower.negate()));
    }

    public static Command extendAndCenterOnChain(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, ClimbSubsystem climbSubsystem) {
        return extendClimber(armSubsystem, climbSubsystem)
        .alongWith(
            Commands.waitSeconds(2)
            .andThen(driveSubsystem.trajectoryToPoseCommand(() -> Field.getNearestChain(driveSubsystem.getFieldPose()), ArrayList::new, true)
        ));
    }

    public static Command lockOnNote(VisionSubsystem visionSubsystem) {
        return new NewDriveCommand(null, null, null, null, null, null, visionSubsystem);
    }

    // private static double voltageChange = 0;

    // public static Command pickupNoteCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
    //      === put into another method  ===
    //     PIDController pid = new PIDController(Drive.turnPIDConfig.kP, Drive.turnPIDConfig.kI, Drive.turnPIDConfig.kD); // Find
    //     pid.setSetpoint(0);
    //     voltageChange = 0.1; // Tune
    //      ==================================
    //     return 
    //     Commands.runOnce(() -> pid.reset())
    //     .andThen(Commands.defer(() -> {
    //         double volts = pid.calculate(visionSubsystem.getNoteYaw());  // gets updated (lowers) every tick
    //         if (Degrees.of(visionSubsystem.getNoteYaw()).lte(Degrees.of(visionSubsystem.getNoteYaw()).plus(Degrees.of(0.5)))) { // checks if the angle to note has decreased
    //             voltageChange += 1; // increases voltage change ammount, thus bringing voltage values closer to the same
    //         }
    //         Measure<Voltage> rightVolts = Volts.of(volts + voltageChange); // in theory, these should approach to be the same as the note yaw gets smaller
    //         Measure<Voltage> leftVolts = Volts.of(-volts + voltageChange);
    //         return driveSubsystem.setVoltageCommand(leftVolts, rightVolts);
    //     }, Set.of(driveSubsystem))
    //     .repeatedly()
    //     .until(intakeShooterSubsystem::isSensorTripped)
    //     .alongWith(Routines.groundIntake(armSubsystem, intakeShooterSubsystem)))
    //     .finallyDo(() -> pid.close());
    // }
}
