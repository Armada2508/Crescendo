// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Joysticks;
import frc.robot.lib.controller.SmartJoystick;
import frc.robot.lib.motion.FollowTrajectory;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class RobotContainer {

    private final SmartJoystick joystick = new SmartJoystick(Joysticks.joystickPort);
    // private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    // private final DriveSubsystem driveSubsystem = new DriveSubsystem(visionSubsystem::getVisionResult);
    // private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final IntakeShooterSubsystem intakeShooterSubsystem = new IntakeShooterSubsystem();
    // private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        FollowTrajectory.config(Drive.ramseteB, Drive.ramseteZeta, Drive.trackWidth);
        // driveSubsystem.setDefaultCommand(driveSubsystem.joystickDriveCommand(
        //     joystick::getYInverted, joystick::getXInverted, joystick::getZInverted
        // ));
        addAutos();
        configureBindings();
    }

    public void stopEverything() {
        CommandScheduler.getInstance().cancelAll();
        // driveSubsystem.stop();
        // armSubsystem.stop();
        // intakeShooterSubsystem.stop();
        // climbSubsystem.stop();
    }

    private void addAutos() {
        // autoChooser.addOption("Leave Starting Zone", Autos.leaveStartingZone(driveSubsystem, armSubsystem));
        // autoChooser.addOption("Score Speaker", Autos.scoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem, pneumaticsSubsystem));
        // autoChooser.addOption("Score Speaker and Leave", Autos.scoreSpeakerAndLeave(driveSubsystem, armSubsystem, intakeShooterSubsystem, pneumaticsSubsystem));
        // autoChooser.addOption("Score Speaker Twice Base", Autos.scoreSpeakerTwiceBase(driveSubsystem, armSubsystem, intakeShooterSubsystem, climbSubsystem, pneumaticsSubsystem));
        // autoChooser.addOption("Score Speaker Twice Side", Autos.scoreSpeakerTwiceSide(driveSubsystem, armSubsystem, intakeShooterSubsystem, pneumaticsSubsystem));
        // autoChooser.setDefaultOption("Score Speaker Thrice", Autos.scoreSpeakerThrice(driveSubsystem, armSubsystem, intakeShooterSubsystem, climbSubsystem, pneumaticsSubsystem));
        SmartDashboard.putData(autoChooser);
    }
    
    private void configureBindings() {
        new Trigger(() -> intakeShooterSubsystem.getShooterVoltage().in(Volts) > 0)
            .onTrue(pneumaticsSubsystem.disableCompressor())
            .onFalse(pneumaticsSubsystem.enableCompressor());
        // joystick.onTrue(1, Routines.turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem, pneumaticsSubsystem));
        // joystick.onTrue(2, Routines.turnToSpeaker(driveSubsystem));
        // joystick.onTrue(3, intakeShooterSubsystem.shootAmpCommand());
        // joystick.onTrue(7, armSubsystem.setAngleCommand(Arm.ampAngle));
        // joystick.onTrue(9, armSubsystem.stowCommand());
        // joystick.onTrue(11, Routines.groundIntake(armSubsystem, intakeShooterSubsystem, pneumaticsSubsystem));
        // joystick.onTrue(5, Commands.runOnce(this::stopEverything));
        // joystick.whileTrue(4, Routines.retractClimber(armSubsystem, climbSubsystem, pneumaticsSubsystem).andThen(climbSubsystem.run(()->{})).finallyDo(climbSubsystem::stop));
        // joystick.whileTrue(6, Routines.extendClimber(armSubsystem, climbSubsystem, pneumaticsSubsystem).andThen(climbSubsystem.run(()->{})).finallyDo(climbSubsystem::stop));
        // joystick.onTrue(10, climbSubsystem.resetClimberCommand());
        // joystick.onTrue(12, Routines.extendAndCenterOnChain(driveSubsystem, armSubsystem, climbSubsystem, pneumaticsSubsystem));

        joystick.onTrue(1, pneumaticsSubsystem.enableCompressor());
        joystick.onTrue(2, pneumaticsSubsystem.disableCompressor());
        joystick.onTrue(4, pneumaticsSubsystem.retract());
        joystick.onTrue(6, pneumaticsSubsystem.extend());
        // Testing shooter map
        // joystick.onTrue(6, armSubsystem.setAngleCommand(armSubsystem.getAngle().plus(Degrees.of(0.5))));
        // joystick.onTrue(4, armSubsystem.setAngleCommand(armSubsystem.getAngle().minus(Degrees.of(0.5))));
        // joystick.onTrue(3, armSubsystem.setAngleCommand(Arm.speakerBaseAngle));
        // joystick.onTrue(1, intakeShooterSubsystem.shootSpeakerCommand());
    }

    public Command getAutonomousCommand() {
        // return climbSubsystem.resetClimberCommand().alongWith(autoChooser.getSelected());
        return Commands.none();
    } 

}
