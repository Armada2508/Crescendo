// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Climb;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Joysticks;
import frc.robot.commands.Autos;
import frc.robot.commands.Routines;
import frc.robot.lib.controller.SmartJoystick;
import frc.robot.lib.motion.FollowTrajectory;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser = new SendableChooser<>();
    private final SmartJoystick joystick = new SmartJoystick(Joysticks.joystickPort);
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(visionSubsystem);
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final IntakeShooterSubsystem intakeShooterSubsystem = new IntakeShooterSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

    public RobotContainer() {
        FollowTrajectory.config(Drive.ramseteB, Drive.ramseteZeta, Drive.trackWidth);
        driveSubsystem.setDefaultCommand(driveSubsystem.joystickDriveCommand(
            joystick::getYInverted, joystick::getXInverted, joystick::getZInverted
        ));
        addAutos();
        configureBindings();
    }

    public void stopEverything() {
        CommandScheduler.getInstance().cancelAll();
        driveSubsystem.stop();
        armSubsystem.stop();
        intakeShooterSubsystem.stop();
        climbSubsystem.stop();
    }

    private void addAutos() {
        autoChooser.setDefaultOption("Leave Starting Zone", Autos.leaveStartingZone(driveSubsystem, armSubsystem));
        autoChooser.addOption("Score Speaker Base and Leave", Autos.scoreSpeakerBaseAndLeave(driveSubsystem, armSubsystem, intakeShooterSubsystem));
        autoChooser.addOption("Score Speaker and Leave", Autos.scoreSpeakerAndLeave(driveSubsystem, armSubsystem, intakeShooterSubsystem));
        autoChooser.addOption("Score Speaker Twice Base", Autos.scoreSpeakerTwiceBase(driveSubsystem, armSubsystem, intakeShooterSubsystem, climbSubsystem));
        autoChooser.addOption("Score Speaker Twice Side", Autos.scoreSpeakerTwiceSide(driveSubsystem, armSubsystem, intakeShooterSubsystem));
        autoChooser.addOption("Score Speaker Thrice", Autos.scoreSpeakerThrice(driveSubsystem, armSubsystem, intakeShooterSubsystem, climbSubsystem));
        SmartDashboard.putData(autoChooser);
    }
    
    private void configureBindings() {
        //! Need to add climber reset to autos and put stuff into routines class
        // Higher arm angle = lower note height
        joystick.onTrue(1, Routines.turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem));
        joystick.onTrue(2, Routines.turnToSpeaker(driveSubsystem));
        // joystick.onTrue(4, armSubsystem.setAngleCommand(Degrees.of(47.75)).andThen(intakeShooterSubsystem.shootSpeakerCommand()).andThen(armSubsystem.stowCommand()));
        joystick.onTrue(3, intakeShooterSubsystem.shootAmpCommand());
        // joystick.onTrue(3, Routines.scoreAmp(driveSubsystem, armSubsystem, intakeShooterSubsystem));
        joystick.onTrue(7, armSubsystem.setAngleCommand(Arm.ampAngle));
        joystick.onTrue(9, armSubsystem.stowCommand());
        joystick.onTrue(11, Routines.groundIntake(armSubsystem, intakeShooterSubsystem));
        joystick.onTrue(5, Commands.runOnce(this::stopEverything));

        joystick.onTrue(20, Routines.centerOnChain(driveSubsystem));
        //Climbing, put into routines class
        joystick.whileTrue(4, armSubsystem.stowCommand().andThen(climbSubsystem.setVoltage(Climb.climbPower.negate()).andThen(climbSubsystem.run(()->{})).finallyDo(climbSubsystem::stop))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        joystick.whileTrue(6, armSubsystem.stowCommand().andThen(climbSubsystem.setVoltage(Climb.climbPower).andThen(climbSubsystem.run(()->{})).finallyDo(climbSubsystem::stop))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        joystick.onTrue(10, climbSubsystem.resetClimberCommand());

        // joystick.onTrue(4, Autos.faceNote(Note.TOP, driveSubsystem));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    } 

}
