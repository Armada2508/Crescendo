package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.Pivot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class CommandRoutines extends SubsystemBase {

    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private PivotSubsystem pivotSubsystem;
    private VisionSubsystem visionSubsystem;

    public CommandRoutines(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, PivotSubsystem pivotSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.pivotSubsystem = pivotSubsystem;
        this.visionSubsystem = visionSubsystem;
    }

    public Command groundIntake() {
        return pivotSubsystem.setAngleCommand(0, 0, 0) //pivot to ground
        .andThen(intakeSubsystem.intakeCommand()) // intake
        .andThen(stowCommand());
    }

    public Command scoreAmp() {
        return pivotSubsystem.setAngleCommand(0, 0, 0) //angle to amp
        .andThen(intakeSubsystem.shootCommand(0)) //shoot slow into amp
        .andThen(stowCommand());
    }

    private Command stowCommand() {
        return runOnce(() -> {
            pivotSubsystem.setAngleCommand(Pivot.stowAngle, 0, 0); //test values
        });
    }

    public Command scoreSpeakerBase() {
        pivotSubsystem.setAngleCommand(CommandConstants.shootAngle, 0, 0).andThen(intakeSubsystem.shootCommand(0)).andThen(stowCommand());
    }

    public Command scoreSpeakerVision() {
        // sqrt(2g * H) / sin(theta)
        double velocity = Math.sqrt(2 * Constants.gravity * CommandConstants.lowSpeakerHeight) / Math.sin(Units.degreesToRadians(CommandConstants.shootAngle));
        pivotSubsystem.setAngleCommand(CommandConstants.shootAngle, 0, 0).andThen(intakeSubsystem.shootCommand(velocity)).andThen(stowCommand());
    }

}
