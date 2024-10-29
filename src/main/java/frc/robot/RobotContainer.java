// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Joysticks;
import frc.robot.commands.Autos;
import frc.robot.commands.Routines;
import frc.robot.lib.controller.Logitech3DPro;
import frc.robot.lib.motion.FollowTrajectory;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    private final Logitech3DPro joystick = new Logitech3DPro(Joysticks.joystickPort);
    private final VisionSubsystem visionSubsystem = new VisionSubsystem();
    private final DriveSubsystem driveSubsystem = new DriveSubsystem(visionSubsystem::getVisionResult);
    private final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final IntakeShooterSubsystem intakeShooterSubsystem = new IntakeShooterSubsystem();
    private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
    private final PneumaticsSubsystem pneumaticsSubsystem = new PneumaticsSubsystem();
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        FollowTrajectory.config(Drive.ramseteB, Drive.ramseteZeta, Drive.trackWidth);
        driveSubsystem.setDefaultCommand(driveSubsystem.joystickDriveCommand(
            joystick::getYInverted, joystick::getXInverted, joystick::getZInverted, joystick.b2(), () -> visionSubsystem.getNoteYaw()
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
        pneumaticsSubsystem.stop();
    }

    private void addAutos() {
        autoChooser.addOption("Leave Starting Zone", climbSubsystem.resetClimberCommand().alongWith(Autos.leaveStartingZone(driveSubsystem, armSubsystem)));
        autoChooser.addOption("Score Speaker", climbSubsystem.resetClimberCommand().alongWith(Autos.scoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem, pneumaticsSubsystem)));
        autoChooser.addOption("Score Speaker and Leave", climbSubsystem.resetClimberCommand().alongWith(Autos.scoreSpeakerAndLeave(driveSubsystem, armSubsystem, intakeShooterSubsystem, pneumaticsSubsystem)));
        autoChooser.addOption("Score Speaker Twice Base", climbSubsystem.resetClimberCommand().alongWith(Autos.scoreSpeakerTwiceBase(driveSubsystem, armSubsystem, intakeShooterSubsystem, climbSubsystem, pneumaticsSubsystem)));
        autoChooser.setDefaultOption("Score Speaker Thrice", climbSubsystem.resetClimberCommand().alongWith(Autos.scoreSpeakerThrice(driveSubsystem, armSubsystem, intakeShooterSubsystem, climbSubsystem, pneumaticsSubsystem)));
        SmartDashboard.putData(autoChooser);
    }
    
    private void configureBindings() {
        new Trigger(() -> intakeShooterSubsystem.getShooterVoltage().in(Volts) > 0)
            .onTrue(pneumaticsSubsystem.disableCompressor())
            .onFalse(pneumaticsSubsystem.enableCompressor());
        joystick.b1().onTrue(Routines.turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem, pneumaticsSubsystem));
        // joystick.b2().onTrue(Routines.turnToSpeaker(driveSubsystem));
        joystick.b3().onTrue(intakeShooterSubsystem.shootAmpCommand());
        // joystick.b5().onTrue(Commands.runOnce(this::stopEverything));
        joystick.b7().onTrue(Routines.ampPosition(armSubsystem, pneumaticsSubsystem));
        joystick.b9().onTrue(Routines.enterStow(armSubsystem, pneumaticsSubsystem));
        joystick.b11().onTrue(Routines.groundIntake(armSubsystem, intakeShooterSubsystem, pneumaticsSubsystem));
        joystick.b4().whileTrue(Routines.retractClimber(armSubsystem, climbSubsystem, pneumaticsSubsystem).andThen(climbSubsystem.run(()->{})).finallyDo(climbSubsystem::stop));
        joystick.b6().whileTrue(Routines.extendClimber(armSubsystem, climbSubsystem, pneumaticsSubsystem).andThen(climbSubsystem.run(()->{})).finallyDo(climbSubsystem::stop));
        joystick.b12().onTrue(Routines.extendAndCenterOnChain(driveSubsystem, armSubsystem, climbSubsystem, pneumaticsSubsystem));

        /*
        * TRIGGER -> SHOOT SPEAKER
        * 3 -> SHOOT AMP
        * 7 -> ENTER AMP SHOOTING POSITION
        * 9 -> ENTER STOW
        * 11 -> GROUND INTAKE
        * 4 -> HOLD RETRACT CLIMBER
        * 6 -> HOLD EXTEND CLIMBER
        * 12 -> AUTO CLIMB

         */

        joystick.b8().onTrue(climbSubsystem.resetClimberCommand());
        // joystick.b10().onTrue(Routines.ampPositionAndCenter(driveSubsystem, armSubsystem, pneumaticsSubsystem));

        // joystick.b4().onTrue(pneumaticsSubsystem.retract());
        // joystick.b6().onTrue(pneumaticsSubsystem.extend());
        // Testing shooter map
        // joystick.b5().onTrue(armSubsystem.setAngleCommand(() -> armSubsystem.getAngle().plus(Degrees.of(1))));
        // joystick.b4().onTrue(armSubsystem.setAngleCommand(() -> armSubsystem.getAngle().minus(Degrees.of(1))));
        // joystick.b3().onTrue(armSubsystem.setAngleCommand(Arm.speakerBaseAngle));
        // joystick.b1().onTrue(intakeShooterSubsystem.shootSpeakerCommand());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    } 

}
