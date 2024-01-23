package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Field;
import frc.robot.Constants.IntakeShooter;
import frc.robot.lib.logging.LogUtil;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

//! Have to find out all the velocities, accelerations for this stuff
public class Routines {

    private Routines() {}
   
    public static Command groundIntake(ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeSubsystem) {
        return armSubsystem.setAngleCommand(Arm.pickupAngle.in(Degrees), 0, 0)
        .andThen(intakeSubsystem.intakeCommand()) 
        .andThen(stowCommand(armSubsystem));
    }

    public static Command scoreAmp(ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem) {
        return armSubsystem.setAngleCommand(Arm.ampAngle.in(Degrees), 0, 0) 
        .andThen(shooterSubsystem.shootCommand(IntakeShooter.ampShootSpeed.in(RotationsPerSecond))) 
        .andThen(stowCommand(armSubsystem));
    }

    public static Command scoreSpeakerBase(ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem) {
        return armSubsystem.setAngleCommand(Arm.speakerAngle.in(Degrees), 0, 0)
        .andThen(shooterSubsystem.shootCommand(IntakeShooter.speakerShootSpeed.in(RotationsPerSecond)))
        .andThen(stowCommand(armSubsystem));
    }

    /**
     * https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)
     */
    public static Command scoreSpeakerVision(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem) {
        return armSubsystem.setAngleCommand(() -> calcAngle(driveSubsystem), 0, 0)
        .andThen(shooterSubsystem.shootCommand(IntakeShooter.speakerShootSpeed.in(RotationsPerSecond)))
        .andThen(stowCommand(armSubsystem));
    }

    public static Command turnToSpeaker(DriveSubsystem driveSubsystem) {
        return driveSubsystem.turnCommand(driveSubsystem.getFieldPose().getRotation().getDegrees() % 180); //? Probably have to fix this
    }

    private static double calcAngle(DriveSubsystem driveSubsystem) {
        // double x = driveSubsystem.getFieldPose().getTranslation().getDistance(Field.speakerPos.getTranslation());
        double x = new Translation2d(Field.speakerPos.getTranslation().getX(), Field.speakerPos.getTranslation().getY() + 2).getDistance(Field.speakerPos.getTranslation());
        double y = Field.lowSpeakerHeight.in(Meters);
        double v = IntakeShooter.speakerShootSpeed.in(RotationsPerSecond) * Math.PI * IntakeShooter.flywheelDiameter.in(Meters);
        double g = Constants.GRAVITY;
        double angle1 = Units.radiansToDegrees(Math.atan( (v * v + inner(x, y, v, g)) / (g * x) ));
        double angle2 = Units.radiansToDegrees(Math.atan( (v * v - inner(x, y, v, g)) / (g * x) ));
        double angle = closer(angle1, angle2, 45);
        LogUtil.printFormatted("X Y V G θ1 θ2 θ", x, y, v, g, angle1, angle2, angle);
        return angle;
    }

    private static double inner(double x, double y, double v, double g) {
        return Math.sqrt(Math.pow(v, 4) - g * (g * x * x + 2 * y * v * v));
    }

    private static double closer(double a, double b, double val) {
        if (Math.abs(a - val) < Math.abs(b - val)) return a;
        return b;
    }   

    private static Command stowCommand(ArmSubsystem armSubsystem) {
        return armSubsystem.setAngleCommand(Arm.stowAngle.in(Degrees), 0, 0);
    }

}
