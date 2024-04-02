package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Drive;
import frc.robot.lib.drive.DriveCommand;

public class NewDriveCommand extends DriveCommand {

    private BooleanSupplier noteModeEnabled;
    private DoubleSupplier noteAngle;
    private final double maxYaw = 30;

    public NewDriveCommand(DoubleSupplier joystickSpeed, 
        DoubleSupplier joystickTurn, 
        DoubleSupplier joystickTrim, 
        DriveConfig config, 
        BiConsumer<Double, Double> speedConsumer, 
        Runnable onEnd,
        Subsystem subsystem,
        BooleanSupplier noteModeEnabled,
        DoubleSupplier noteAngle) {
        super(joystickSpeed, joystickTurn, joystickTrim, () -> false, config, speedConsumer, onEnd, subsystem);
        this.noteModeEnabled = noteModeEnabled;
        this.noteAngle = noteAngle;
    }

    @Override
    public void execute() {
        double speed = joystickSpeed.getAsDouble();
        double turn = joystickTurn.getAsDouble();
        double trim = joystickTrim.getAsDouble();
        // Deadband
        speed = processDeadband(speed);
        turn = processDeadband(turn); 
        trim = processDeadband(trim); 
        // Square the inputs
        if (config.squareInputs()) {
            speed = Math.signum(speed) * (speed * speed);
            turn = Math.signum(turn) * (turn * turn);        
            trim = Math.signum(trim) * (trim * trim);   
        }
        // Fade out trim
        trim *= MathUtil.clamp(1 - Math.abs(speed * Drive.trimFadeout), 0, 1);
        // Speed Adjusting and Slew Rate Limiting 
        speed *= config.speedAdjustment();
        turn *= config.turnAdjustment();
        trim *= config.trimAdjustment();
        speed = limiter.calculate(speed);
        // Constant Curvature, WPILib DifferentialDrive#curvatureDriveIK
        if (config.constantCurvature()) {
            turn = turn * speed + trim; 
        }

        if (noteModeEnabled.getAsBoolean()) {
            turn += noteAngle.getAsDouble() / (maxYaw * 2);
        }

        double powerFactor = normalizeSpeed((speed - turn), (speed + turn));

        double leftSpeed = (speed - turn) * powerFactor;
        double rightSpeed = (speed + turn) * powerFactor;
        speedConsumer.accept(leftSpeed, rightSpeed);
    }

}
