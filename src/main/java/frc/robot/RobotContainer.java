// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Field;
import frc.robot.Constants.Joysticks;
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
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(visionSubsystem::getEstimatedGlobalPose);
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final IntakeShooterSubsystem intakeShooterSubsystem = new IntakeShooterSubsystem();

    public RobotContainer() {
        FollowTrajectory.config(0, 0, 0, 2.0, 0.7, Drive.trackWidth.in(Meters), new PIDController(0, 0, 0), 0); //! Constants!
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
        // joystick.onTrue(11, getTrajectoryCommand(new Pose2d(Meters.of(5), Meters.of(5), Rotation2d.fromDegrees(0))));
        joystick.onTrue(11, getTrajectoryCommand(new Pose2d(Field.speakerPos, Rotation2d.fromDegrees(180))));
    }

    public Command getTrajectoryCommand(Pose2d targetPose) {
        TrajectoryConfig config = new TrajectoryConfig(MetersPerSecond.of(0.75), MetersPerSecondPerSecond.of(0.5));
        config.setKinematics(Drive.diffKinematics);
        // Pose2d start = driveSubsystem.getFieldPose();
        Pose2d start = new Pose2d(Meters.of(5), Meters.of(3), Rotation2d.fromDegrees(180));
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, new ArrayList<>(), targetPose, config);
        Field.simulatedField.getObject("traj").setTrajectory(trajectory);
        return FollowTrajectory.getCommandTalon(trajectory, new Pose2d(), driveSubsystem::getFieldPose, driveSubsystem::setVelocity, driveSubsystem);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private DoubleSupplier reverseAxisIf(DoubleSupplier axis, int button) {
        return () -> buttonBoard.getRawButton(button) ? -axis.getAsDouble() : axis.getAsDouble();
    }

}
