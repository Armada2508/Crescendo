package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Field;
import frc.robot.Constants.Pivot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class Routines {

    private Routines() {}
   
    public static Command groundIntake(PivotSubsystem pivotSubsystem, IntakeSubsystem intakeSubsystem) {
        return pivotSubsystem.setAngleCommand(0, 0, 0) //pivot to ground, test velocity and acceleration values
        .andThen(intakeSubsystem.intakeCommand()) // intake
        .andThen(stowCommand(pivotSubsystem));
    }

    public static Command scoreAmp(PivotSubsystem pivotSubsystem, IntakeSubsystem intakeSubsystem) {
        return pivotSubsystem.setAngleCommand(0, 0, 0) //angle to amp, test velocity and acceleration values
        .andThen(intakeSubsystem.shootCommand(0)) //shoot slow into amp
        .andThen(stowCommand(pivotSubsystem));
    }

    public static Command scoreSpeakerBase(PivotSubsystem pivotSubsystem, IntakeSubsystem intakeSubsystem) {
        return pivotSubsystem.setAngleCommand(Pivot.shootAngle, 0, 0).andThen(intakeSubsystem.shootCommand(0)).andThen(stowCommand(pivotSubsystem));
    }

    public static Command scoreSpeakerVision(PivotSubsystem pivotSubsystem, IntakeSubsystem intakeSubsystem, VisionSubsystem visionSubsystem) {
        // sqrt(2g * H) / sin(theta)
        double velocity = Math.sqrt(2 * Constants.gravity * Field.lowSpeakerHeight) / Math.sin(Units.degreesToRadians(Pivot.shootAngle));
        return pivotSubsystem.setAngleCommand(Pivot.shootAngle, 0, 0).andThen(intakeSubsystem.shootCommand(velocity)).andThen(stowCommand(pivotSubsystem));
    }

    private static Command stowCommand(PivotSubsystem pivotSubsystem) {
        return pivotSubsystem.setAngleCommand(Pivot.stowAngle, 0, 0); //test velocity and acceleration values
    }
}
