package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class CommandRoutines extends SubsystemBase{

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
        return pivotSubsystem.setAngleCommand(0, 0, 0) //pivot to groudn
        .andThen(intakeSubsystem.intakeCommand()) // intake
        .andThen(pivotSubsystem.setAngleCommand(0, 0, 0)); 
    }

    public Command scoreAmp() {
        return runOnce(() -> {
            pivotSubsystem.setAngleCommand(0, 0, 0); //pivot to amp
            intakeSubsystem.shootCommand(0); //shoot show into amp
            pivotSubsystem.setAngleCommand(0, 0, 0); //stow
        });
    }
}
