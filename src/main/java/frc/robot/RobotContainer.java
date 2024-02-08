// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Joysticks;
import frc.robot.commands.Autos;
import frc.robot.lib.controller.SmartJoystick;
import frc.robot.lib.motion.FollowTrajectory;
import frc.robot.lib.util.Util;
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
        {   // Testing shooter prototype
            intakeShooterSubsystem.setDefaultCommand(intakeShooterSubsystem.run(() -> {
                double val = joystick.getY();
                if (Util.inRange(val, Drive.joystickDriveConfig.joystickDeadband())) {
                    val = 0;
                }
                intakeShooterSubsystem.setShooterSpeed(val);
            }));
        }
        buttonBoard.onTrue(Joysticks.solenoidButton, armSubsystem.runOnce(() -> armSubsystem.switchRelay()));
        
        // joystick.onTrue(11, Routines.scoreAmp(armSubsystem, intakeShooterSubsystem));
        // joystick.onTrue(10, Commands.runOnce(this::stopEverything, new Subsystem[]{}));
        // new Trigger(() -> true).onTrue(intakeShooterSubsystem.run(() -> {
        //     intakeShooterSubsystem.setShooterSpeed(-joystick.getY());
        // }));
        // joystick.whileTrue(5, intakeShooterSubsystem.runOnce(() -> intakeShooterSubsystem.setSpeed(1)).finallyDo(intakeShooterSubsystem::stop));
        // joystick.whileTrue(6, armSubsystem.runOnce(() -> intakeShooterSubsystem.setSpeed(.25)).finallyDo(armSubsystem::stop));
        // joystick.whileTrue(4, armSubsystem.runOnce(() -> intakeShooterSubsystem.setSpeed(-.25)).finallyDo(armSubsystem::stop));
        // joystick.onTrue(8, driveSubsystem.turnCommand(-45));
        // joystick.onTrue(9, driveSubsystem.turnCommand(45));
        // joystick.onTrue(10, driveSubsystem.driveDistanceCommand(-2, 2, 2));
        // joystick.onTrue(11, driveSubsystem.driveDistanceCommand(-2, 2, 2));
        // joystick.onTrue(7, Routines.scoreSpeakerVision(driveSubsystem, armSubsystem, intakeShooterSubsystem));
        // joystick.onTrue(11, getTrajectoryCommand(new Pose2d(Meters.of(5), Meters.of(5), Rotation2`d.fromDegrees(0))));
        // joystick.onTrue(11, driveSubsystem.trajectoryToPoseCommand(new Pose2d(Field.speakerPos, Rotation2d.fromDegrees(180))));
    }

    public Command getAutonomousCommand() {
        return armSubsystem.relayStartOfMatchCommand()
        .andThen(Autos.leaveStartingZone(driveSubsystem));
    } 

    private DoubleSupplier reverseAxisIf(DoubleSupplier axis, int button) {
        return () -> buttonBoard.getRawButton(button) ? axis.getAsDouble() : -axis.getAsDouble();
    }

    public static GenericEntry getTuner(String name, Object defaultValue) {
        GenericEntry entry = NetworkTableInstance.getDefault().getTable("Tuning")
            .getTopic("Ratio").getGenericEntry(new PubSubOption[0]);
        entry.setValue(defaultValue);
        return entry;
    }

}
