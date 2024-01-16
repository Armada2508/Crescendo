// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.controller.SmartJoystick;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    private final SmartJoystick joystick = new SmartJoystick(0);
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(visionSubsystem::getEstimatedGlobalPose);
    private final PivotSubsystem pivotSubsystem = new PivotSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    public RobotContainer() {
        driveSubsystem.setDefaultCommand(driveSubsystem.joystickDriveCommand(joystick::getY, joystick::getX, joystick::getZ, () -> joystick.getRawButton(12)));
        configureBindings();
    }

    public void stopEverything() {
        CommandScheduler.getInstance().cancelAll();
        driveSubsystem.stop();
        pivotSubsystem.stop();
        intakeSubsystem.stop();
    }
    
    private void configureBindings() {
        
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

}
