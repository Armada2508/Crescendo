package frc.robot.commands;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Field;
import frc.robot.Constants.Shooter;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

//! Have to find out all the velocities, accelerations for this stuff
public class Routines {

    private Routines() {}
   
    public static Command groundIntake(ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeSubsystem) {
        return armSubsystem.setAngleCommand(Arm.pickupAngle, 0, 0, 0)
        .andThen(
            intakeSubsystem.intakeCommand(),
            stowCommand(armSubsystem)
        ).withName("Intake Ground Composition");
    }

    public static Command scoreAmp(ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem) {
        return armSubsystem.setAngleCommand(Arm.ampAngle, 0, 0, 0) 
        .alongWith(shooterSubsystem.spinUpFlywheelCommand(Shooter.ampShootSpeed))
        .andThen(
            shooterSubsystem.releaseNoteCommand(),
            stowCommand(armSubsystem)
        )    
        .withName("Score Amp Composition");
    }

    public static Command scoreSpeakerBase(ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem) {
        return armSubsystem.setAngleCommand(Arm.speakerAngle, 0, 0, 0)
        .alongWith(shooterSubsystem.spinUpFlywheelCommand(Shooter.speakerShootSpeed))
        .andThen(
            shooterSubsystem.releaseNoteCommand(),
            stowCommand(armSubsystem)
        )    
        .withName("Score Speaker Base Composition");
    }

    public static Command scoreSpeakerVision(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem) {
        return armSubsystem.setAngleCommand(() -> getAngle(driveSubsystem, armSubsystem), 0, 0, 0)
        .alongWith(shooterSubsystem.spinUpFlywheelCommand(Shooter.speakerShootSpeed))
        .andThen(
            shooterSubsystem.releaseNoteCommand(),
            stowCommand(armSubsystem)
        )    
        .withName("Score Speaker Vision Composition");
    }

    public static Command turnToSpeaker(DriveSubsystem driveSubsystem) {
        return driveSubsystem.turnCommand(
            // Radians.of(Math.atan2((driveSubsystem.getFieldPose().getY() - Field.speakerPos.getY()), -driveSubsystem.getFieldPose().getX() - Field.speakerPos.getX()))
            () -> {
                Translation2d speakerPos = (Robot.onRedAlliance()) ? Field.redSpeakerPosition : Field.blueSpeakerPosition;
                return Radians.of(driveSubsystem.getFieldPose().getTranslation().minus(speakerPos).getAngle().getRadians());
            }
        )
        .withName("Turn to Speaker Command");
    }

    public static Command stowCommand(ArmSubsystem armSubsystem) {
        return armSubsystem.setAngleCommand(Arm.stowAngle, 0, 0, 0).withName("Stow Command");
    }

    private static Measure<Angle> getAngle(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {
        Translation2d speakerPos = (Robot.onRedAlliance()) ? Field.redSpeakerPosition : Field.blueSpeakerPosition;
        double distance = driveSubsystem.getFieldPose().getTranslation().getDistance(speakerPos);
        return armSubsystem.getTargetAngle(distance);
    }

    /**
     * https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)
     */
    // private static Measure<Angle> calcAngle(DriveSubsystem driveSubsystem) {
    //     double x = driveSubsystem.getFieldPose().getTranslation().getDistance(Field.speakerPos);
    //     // double x = new Translation2d(Field.speakerPos.getX(), Field.speakerPos.getY() + 2).getDistance(Field.speakerPos);
    //     double y = Field.lowSpeakerHeight.in(Meters);
    //     double v = Shooter.speakerShootSpeed.in(RotationsPerSecond) * Math.PI * Shooter.flywheelDiameter.in(Meters);
    //     double g = Constants.gravity;
    //     double angle1 = Units.radiansToDegrees(Math.atan( (v * v + inner(x, y, v, g)) / (g * x) ));
    //     double angle2 = Units.radiansToDegrees(Math.atan( (v * v - inner(x, y, v, g)) / (g * x) ));
    //     double angle = closer(angle1, angle2, 45); //? Magic number also might be garbage
    //     LogUtil.printFormatted("X Y V G θ1 θ2 θ", x, y, v, g, angle1, angle2, angle);
    //     return angle;
    // }

    // private static double inner(double x, double y, double v, double g) {
    //     return Math.sqrt(Math.pow(v, 4) - g * (g * x * x + 2 * y * v * v));
    // }

    // private static double closer(double a, double b, double val) {
    //     if (Math.abs(a - val) < Math.abs(b - val)) return a;
    //     return b;
    // }   
    
}
