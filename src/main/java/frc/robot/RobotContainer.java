// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Joysticks;
import frc.robot.commands.Autos;
import frc.robot.lib.controller.SmartJoystick;
import frc.robot.lib.motion.FollowTrajectory;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    private final SmartJoystick joystick = new SmartJoystick(Joysticks.joystickPort);
    private final SmartJoystick buttonBoard = new SmartJoystick(Joysticks.buttonBoardPort);
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(visionSubsystem);
    // private final ArmSubsystem armSubsystem = new ArmSubsystem();
    // private final IntakeShooterSubsystem intakeShooterSubsystem = new IntakeShooterSubsystem();

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
        // armSubsystem.stop();
        // intakeShooterSubsystem.stop();
    }
    
    private void configureBindings() {
        joystick.onTrue(2, driveSubsystem.turnCommand(() -> {
            return Degrees.of(driveSubsystem.getFieldPose().getRotation().getDegrees() - visionSubsystem.getNoteYaw());
        }));
        joystick.onTrue(3, driveSubsystem.startEnd(() -> {
            driveSubsystem.setSpeed(.15, .15);
        }, driveSubsystem::stop).until(
            () -> Math.abs(visionSubsystem.getNotePitch()) > 15
        ));
        // joystick.onTrue(5, intakeShooterSubsystem.spinUpFlywheelCommand(Shooter.speakerShootPower));
        // joystick.onTrue(3, intakeShooterSubsystem.spinUpFlywheelCommand(Shooter.ampShootPower));
        // buttonBoard.onTrue(Joysticks.solenoidButton, armSubsystem.runOnce(() -> armSubsystem.switchRelay()));
        // joystick.onTrue(3, driveSubsystem.runOnce(() -> {
        //     driveSubsystem.setVelocity(-.25, -.25);
        // }));
        joystick.onTrue(4, driveSubsystem.runOnce(() -> {
            driveSubsystem.resetFieldPose();
        }));
        // joystick.onTrue(5, driveSubsystem.runOnce(() -> {
        //     driveSubsystem.setVelocity(.25, .25);
        // }));
        // joystick.onTrue(6, driveSubsystem.trajectoryToPoseCommand(
        //     () -> (Robot.onRedAlliance()) ? new Pose2d(Field.redSpeakerPosition, Rotation2d.fromDegrees(0)) : new Pose2d(Field.blueSpeakerPosition, Rotation2d.fromDegrees(180))
        // ));
        joystick.onTrue(7, driveSubsystem.trajectoryToPoseCommand(new Pose2d(Feet.of(5), Feet.of(3), Rotation2d.fromDegrees(0))));
        joystick.onTrue(9, driveSubsystem.turnCommand(Degrees.of(45)));
        joystick.onTrue(10, driveSubsystem.turnCommand(Degrees.of(-45)));
        joystick.onTrue(11, driveSubsystem.turnCommand(Degrees.of(90)));
        // joystick.onTrue(10, driveSubsystem.driveDistanceCommand(Feet.of(-2), 2, 2));
        // joystick.onTrue(11, driveSubsystem.driveDistanceCommand(Feet.of(2), 2, 2));
        joystick.onTrue(1, Commands.runOnce(this::stopEverything));
    }

    public Command getAutonomousCommand() {
        return /*armSubsystem.relayStartOfMatchCommand()
        .andThen*/(Autos.leaveStartingZone(driveSubsystem));
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
