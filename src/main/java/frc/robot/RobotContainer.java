// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Field;
import frc.robot.Constants.Joysticks;
import frc.robot.commands.Autos;
import frc.robot.lib.controller.SmartJoystick;
import frc.robot.lib.motion.FollowTrajectory;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    private final SmartJoystick joystick = new SmartJoystick(Joysticks.joystickPort);
    private final SmartJoystick buttonBoard = new SmartJoystick(Joysticks.buttonBoardPort);
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(visionSubsystem);
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final IntakeShooterSubsystem intakeShooterSubsystem = new IntakeShooterSubsystem();

    public RobotContainer() {
        FollowTrajectory.config(Drive.ramseteB, Drive.ramseteZeta, Drive.trackWidth);
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
        joystick.onTrue(7, driveSubsystem.trajectoryToPoseCommand(() -> Robot.onRedAlliance() ? Field.redSpeakerBaseScorePos : Field.blueSpeakerBaseScorePos, true));
        joystick.onTrue(9, driveSubsystem.turnCommand(Degrees.of(45)));
        joystick.onTrue(10, driveSubsystem.turnCommand(Degrees.of(-45)));
        joystick.onTrue(11, driveSubsystem.turnCommand(Degrees.of(90)));
        // joystick.onTrue(10, driveSubsystem.driveDistanceCommand(Feet.of(-2), 2, 2));
        // joystick.onTrue(11, driveSubsystem.driveDistanceCommand(Feet.of(2), 2, 2));
        joystick.onTrue(1, Commands.runOnce(this::stopEverything));
    }

    public Command getAutonomousCommand() {
        return Autos.leaveStartingZone(driveSubsystem);
    } 

    private DoubleSupplier reverseAxisIf(DoubleSupplier axis, int button) {
        return () -> buttonBoard.getRawButton(button) ? axis.getAsDouble() : -axis.getAsDouble();
    }

    public static GenericEntry getTuner(String name, Object defaultValue) {
        GenericEntry entry = NetworkTableInstance.getDefault().getTable("Tuning")
            .getTopic(name).getGenericEntry(new PubSubOption[0]);
        entry.setValue(defaultValue);
        return entry;
    }

}
