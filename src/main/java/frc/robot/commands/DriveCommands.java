package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Driving;
import frc.robot.Constants.Vision;
import frc.robot.lib.drive.DriveUtil;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommands {

    public static Command drive(DoubleSupplier joystickSpeed, DoubleSupplier joystickTurn, DoubleSupplier joystickTrim, 
            BooleanSupplier noteModeEnabled, DoubleSupplier noteAngle, DriveSubsystem driveSubsystem) {
        return Commands.runEnd(() -> {
            double speed = joystickSpeed.getAsDouble();
            double turn = joystickTurn.getAsDouble();
            double trim = joystickTrim.getAsDouble();

            speed = DriveUtil.processDeadband(speed, Driving.joystickDeadband, Driving.deadbandSmoothing);
            turn = DriveUtil.processDeadband(turn, Driving.joystickDeadband, Driving.deadbandSmoothing); 
            trim = DriveUtil.processDeadband(trim, Driving.joystickDeadband, Driving.deadbandSmoothing); 

            if (Driving.squareInputs) {
                speed = DriveUtil.squareInput(speed);
                turn = DriveUtil.squareInput(turn);        
                trim = DriveUtil.squareInput(trim);   
            }
            // Fade out trim
            trim *= MathUtil.clamp(1 - Math.abs(speed * Driving.trimFadeout), 0, 1);

            speed *= Driving.speedAdjustment;
            turn *= Driving.turnAdjustment;
            trim *= Driving.trimAdjustment;

            if (Driving.constantCurvature) {
                turn = DriveUtil.constantCurvature(speed, turn, trim); 
            }

            if (noteModeEnabled.getAsBoolean()) {
                turn += noteAngle.getAsDouble() / (Vision.maxYaw.in(Degrees) * 2);
            }

            double leftSpeed = (speed - turn);
            double rightSpeed = (speed + turn);
            Pair<Double, Double> speeds = DriveUtil.normalizeValues(leftSpeed, rightSpeed);
            driveSubsystem.setSpeed(speeds.getFirst(), speeds.getSecond());
        }, driveSubsystem::stop, driveSubsystem); 
    }

}
