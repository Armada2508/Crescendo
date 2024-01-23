// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Joysticks;
import frc.robot.lib.controller.SmartJoystick;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    private final SmartJoystick joystick = new SmartJoystick(Joysticks.joystickPort);
    private final SmartJoystick buttonBoard = new SmartJoystick(Joysticks.buttonBoardPort);
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(visionSubsystem::getEstimatedGlobalPose);
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final IntakeShooterSubsystem intakeShooterSubsystem = new IntakeShooterSubsystem();

    public RobotContainer() {
        joystick.bindButtons(Joysticks.driveSlowButton); 
        buttonBoard.bindButtons(Joysticks.driveReverseButton); 
        driveSubsystem.setDefaultCommand(driveSubsystem.joystickDriveCommand(
            reverseAxisIf(joystick::getY, Joysticks.driveReverseButton), reverseAxisIf(joystick::getX, Joysticks.driveReverseButton), 
            reverseAxisIf(joystick::getZ, Joysticks.driveReverseButton), () -> joystick.getRawButton(Joysticks.driveSlowButton)
        ));
        configureBindings();
    }

    public void stopEverything() {
        CommandScheduler.getInstance().cancelAll();
        driveSubsystem.stop();
        armSubsystem.stop();
        intakeShooterSubsystem.stop();
    }
    
    private void configureBindings() {
        // joystick.whileTrue(5, intakeShooterSubsystem.runOnce(() -> intakeShooterSubsystem.setSpeed(1)).finallyDo(intakeShooterSubsystem::stop));
        // joystick.whileTrue(6, armSubsystem.runOnce(() -> intakeShooterSubsystem.setSpeed(.25)).finallyDo(armSubsystem::stop));
        // joystick.whileTrue(4, armSubsystem.runOnce(() -> intakeShooterSubsystem.setSpeed(-.25)).finallyDo(armSubsystem::stop));
        // joystick.onTrue(8, driveSubsystem.turnCommand(-45));
        // joystick.onTrue(9, driveSubsystem.turnCommand(45));
        // joystick.onTrue(10, driveSubsystem.driveDistanceCommand(-2, 2, 2));
        // joystick.onTrue(11, driveSubsystem.driveDistanceCommand(-2, 2, 2));
        // joystick.onTrue(7, Routines.scoreSpeakerVision(driveSubsystem, armSubsystem, intakeShooterSubsystem));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private DoubleSupplier reverseAxisIf(DoubleSupplier axis, int button) {
        return () -> buttonBoard.getRawButton(button) ? -axis.getAsDouble() : axis.getAsDouble();
    }

}
