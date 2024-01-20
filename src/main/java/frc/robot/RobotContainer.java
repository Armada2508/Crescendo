// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.controller.SmartJoystick;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NoteManipulationSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    private final SmartJoystick joystick = new SmartJoystick(0);
    private final SmartJoystick buttonBoard = new SmartJoystick(1);
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(visionSubsystem::getEstimatedGlobalPose);
    // private final DriveSubsystem driveSubsystem = new DriveSubsystem(Optional::empty);
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final NoteManipulationSubsystem intakeSubsystem = new NoteManipulationSubsystem();

    public RobotContainer() {
        int reverseButton = 1;
        joystick.bindButtons(12); // Drive Slow
        buttonBoard.bindButtons(reverseButton); // Drive Reverse
        driveSubsystem.setDefaultCommand(driveSubsystem.joystickDriveCommand(
            reverseAxisIf(joystick::getY, reverseButton), reverseAxisIf(joystick::getX, reverseButton), reverseAxisIf(joystick::getZ, reverseButton), () -> joystick.getRawButton(12)
        ));
        configureBindings();
    }

    public void stopEverything() {
        CommandScheduler.getInstance().cancelAll();
        driveSubsystem.stop();
        armSubsystem.stop();
        intakeSubsystem.stop();
    }
    
    private void configureBindings() {
        // joystick.whileTrue(5, intakeSubsystem.runOnce(() -> intakeSubsystem.setSpeed(1)).finallyDo(intakeSubsystem::stop));
        // joystick.whileTrue(6, armSubsystem.runOnce(() -> intakeSubsystem.setSpeed(.25)).finallyDo(armSubsystem::stop));
        // joystick.whileTrue(4, armSubsystem.runOnce(() -> intakeSubsystem.setSpeed(-.25)).finallyDo(armSubsystem::stop));
        // joystick.onTrue(8, driveSubsystem.turnCommand(-45));
        // joystick.onTrue(9, driveSubsystem.turnCommand(45));
        // joystick.onTrue(10, driveSubsystem.driveDistanceCommand(-2, 2, 2));
        // joystick.onTrue(11, driveSubsystem.driveDistanceCommand(-2, 2, 2));
        joystick.onTrue(7, Routines.scoreSpeakerVision(driveSubsystem, armSubsystem, intakeSubsystem));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private DoubleSupplier reverseAxisIf(DoubleSupplier axis, int button) {
        return () -> buttonBoard.getRawButton(button) ? -axis.getAsDouble() : axis.getAsDouble();
    }

}
