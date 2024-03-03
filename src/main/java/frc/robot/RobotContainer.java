// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Climb;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Joysticks;
import frc.robot.commands.Autos;
import frc.robot.commands.Routines;
import frc.robot.lib.Encoder;
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
        autoChooser.addOption("Score Speaker Twice Base", Autos.scoreSpeakerTwiceBase(driveSubsystem, armSubsystem, intakeShooterSubsystem));
        autoChooser.addOption("Score Speaker Twice Side", Autos.scoreSpeakerTwiceSide(driveSubsystem, armSubsystem, intakeShooterSubsystem));
        autoChooser.addOption("Score Speaker Thrice", Autos.scoreSpeakerThrice(driveSubsystem, armSubsystem, intakeShooterSubsystem));
        SmartDashboard.putData(autoChooser);
    }
    
    private void configureBindings() {
        // Higher arm angle = lower note height
        joystick.onTrue(1, Routines.turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem));
        joystick.onTrue(2, Routines.turnToSpeaker(driveSubsystem));
        // joystick.onTrue(4, armSubsystem.setAngleCommand(Degrees.of(50)).andThen(intakeShooterSubsystem.shootSpeakerCommand()).andThen(Routines.stowCommand(armSubsystem)));
        joystick.onTrue(3, intakeShooterSubsystem.shootAmpCommand());
        // joystick.onTrue(3, Routines.scoreAmp(driveSubsystem, armSubsystem, intakeShooterSubsystem));
        // joystick.onTrue(7, armSubsystem.setAngleCommand(Arm.ampAngle));
        joystick.onTrue(9, armSubsystem.stowCommand());
        // joystick.onTrue(11, Routines.groundIntake(armSubsystem, intakeShooterSubsystem));
        joystick.onTrue(6, Commands.runOnce(this::stopEverything));

        joystick.whileTrue(10, climbSubsystem.setVoltage(Climb.climbPower.negate()).andThen(climbSubsystem.run(()->{})).finallyDo(climbSubsystem::stop));
        joystick.whileTrue(11, climbSubsystem.setVoltage(Climb.climbPower).andThen(climbSubsystem.run(()->{})).finallyDo(climbSubsystem::stop));
        joystick.onTrue(12, climbSubsystem.resetClimberCommand());
        // joystick.onTrue(4, driveSubsystem.setVelocityCommand(FeetPerSecond.of(6), FeetPerSecond.of(6)).andThen(driveSubsystem.run(() -> {})));
        // joystick.onTrue(4, driveSubsystem.motionMagicVelocityCommand(FeetPerSecond.of(6), FeetPerSecond.of(6), FeetPerSecond.per(Second).of(3)).andThen(driveSubsystem.run(() -> {})));

        
        // System.out.println("Center " + Units.radiansToDegrees(Vision.aprilTagFieldLayout.getTagPose(14).get().getRotation().getZ()));
        // System.out.println("Left " + Units.radiansToDegrees(Vision.aprilTagFieldLayout.getTagPose(15).get().getRotation().getZ()));
        // System.out.println("Right" + Units.radiansToDegrees(Vision.aprilTagFieldLayout.getTagPose(16).get().getRotation().getZ()));


        // STAGE Center, 14, X: 5.32, Y: 4.11, Z: 1.32, angle = 0 
        // STAGE Left, 15, X: 4.64, Y: 4.50, Z: 1.32, angle = 120
        // STAGE Right, 16, X: 4.64, Y: 3.71, Z: 1.32, angle = -120
       
        // joystick.onTrue(10, armSubsystem.initArmAngle());
        joystick.onTrue(10, driveSubsystem.setVelocityCommand(FeetPerSecond.of(6), FeetPerSecond.of(6)).andThen(driveSubsystem.run(() -> {})));
        System.out.println(Encoder.fromDistance(1, Drive.gearRatio, Drive.wheelDiameter.in(Feet)));
        // 6.679050868953597
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    } 

}
