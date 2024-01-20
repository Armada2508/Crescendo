package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Field;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Pivot;
import frc.robot.lib.logging.LogUtil;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

//! Have to find out all the velocities, accelerations for this stuff
public class Routines {

    private Routines() {}
   
    public static Command groundIntake(PivotSubsystem pivotSubsystem, IntakeSubsystem intakeSubsystem) {
        return pivotSubsystem.setAngleCommand(Pivot.pickupAngle, 0, 0) // pivot to ground
        .andThen(intakeSubsystem.intakeCommand()) // intake
        .andThen(stowCommand(pivotSubsystem));
    }

    public static Command scoreAmp(PivotSubsystem pivotSubsystem, IntakeSubsystem intakeSubsystem) {
        return pivotSubsystem.setAngleCommand(Pivot.ampAngle, 0, 0) // pivot to amp
        .andThen(intakeSubsystem.shootCommand(Intake.ampShootSpeed)) // shoot into amp
        .andThen(stowCommand(pivotSubsystem));
    }

    public static Command scoreSpeakerBase(PivotSubsystem pivotSubsystem, IntakeSubsystem intakeSubsystem) {
        return pivotSubsystem.setAngleCommand(Pivot.speakerAngle, 0, 0)
        .andThen(intakeSubsystem.shootCommand(Intake.speakerShootSpeed))
        .andThen(stowCommand(pivotSubsystem));
    }

    /**
     * https://en.wikipedia.org/wiki/Projectile_motion#Angle_%CE%B8_required_to_hit_coordinate_(x,_y)
     */
    public static Command scoreSpeakerVision(DriveSubsystem driveSubsystem, PivotSubsystem pivotSubsystem, IntakeSubsystem intakeSubsystem) {
        return pivotSubsystem.setAngleCommand(() -> calcAngle(driveSubsystem), 0, 0)
        .andThen(intakeSubsystem.shootCommand(Intake.speakerShootSpeed))
        .andThen(stowCommand(pivotSubsystem));
    }

    public static Command turnToSpeaker(DriveSubsystem driveSubsystem) {
        return driveSubsystem.turnCommand(driveSubsystem.getFieldPose().getRotation().getDegrees() % 180); //? Probably have to fix this
    }

    private static double calcAngle(DriveSubsystem driveSubsystem) {
        // double x = driveSubsystem.getFieldPose().getTranslation().getDistance(Field.speakerPos.getTranslation());
        double x = new Translation2d(Field.speakerPos.getTranslation().getX(), Field.speakerPos.getTranslation().getY() + 2).getDistance(Field.speakerPos.getTranslation());
        double y = Field.lowSpeakerHeight;
        double v = Intake.speakerShootSpeed * Math.PI * Intake.flywheelDiamter;
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

    private static Command stowCommand(PivotSubsystem pivotSubsystem) {
        return pivotSubsystem.setAngleCommand(Pivot.stowAngle, 0, 0); //test velocity and acceleration values
    }
}
