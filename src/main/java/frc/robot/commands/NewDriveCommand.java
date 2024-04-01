package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.Drive;
import frc.robot.lib.drive.DriveCommand;

public class NewDriveCommand extends DriveCommand {

    private boolean noteModeEnabled = false;
    private double noteAngle = 0;
    private final double maxNoteYaw = 60; //? Tune

    public NewDriveCommand(DoubleSupplier joystickSpeed, 
    DoubleSupplier joystickTurn, 
    DoubleSupplier joystickTrim, 
            DriveConfig config, 
            BiConsumer<Double, Double> speedConsumer, 
            Runnable onEnd,
            Subsystem subsystem) {
        super(joystickSpeed, joystickTurn, joystickTrim, () -> false, config, speedConsumer, onEnd, subsystem);
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

        if (noteModeEnabled) {
            double extraTurn = noteAngle / (maxNoteYaw * maxNoteYaw);
            turn += extraTurn;
        }

        double powerFactor = normalizeSpeed((speed - turn), (speed + turn));

        double leftSpeed = (speed - turn) * powerFactor;
        double rightSpeed = (speed + turn) * powerFactor;
        speedConsumer.accept(leftSpeed, rightSpeed);
    }

}
