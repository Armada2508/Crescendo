// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.controller.SmartJoystick;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    private final SmartJoystick joystick = new SmartJoystick(0);
    private final SmartJoystick buttonBoard = new SmartJoystick(1);
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(visionSubsystem::getEstimatedGlobalPose);
    private final PivotSubsystem pivotSubsystem = new PivotSubsystem();

    public RobotContainer() {
        int reverseButton = 0;
        joystick.bindButtons(12); // Drive Slow
        buttonBoard.bindButtons(reverseButton); // Drive Reverse
        driveSubsystem.setDefaultCommand(driveSubsystem.joystickDriveCommand(
            reverseAxisIf(joystick::getY, reverseButton), reverseAxisIf(joystick::getX, reverseButton), reverseAxisIf(joystick::getY, reverseButton), () -> joystick.getRawButton(12)
        ));
        configureBindings();
    }

    public void stopEverything() {
        CommandScheduler.getInstance().cancelAll();
        driveSubsystem.stop();
        pivotSubsystem.stop();
    }
    
    private void configureBindings() {
        
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private DoubleSupplier reverseAxisIf(DoubleSupplier axis, int button) {
        return () -> buttonBoard.getRawButton(button) ? -axis.getAsDouble() : axis.getAsDouble();
    }

}
