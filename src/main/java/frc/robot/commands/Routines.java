package frc.robot.commands;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.Constants.FeetPerSecondSquared;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Climb;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Pneumatics;
import frc.robot.Constants.Shooter;
import frc.robot.Field;
import frc.robot.Robot;
import frc.robot.lib.util.Util;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class Routines {

    private Routines() {}

    public static Command enterStow(ArmSubsystem armSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return Commands.either(pneumaticsSubsystem.extend(), Commands.none(), () -> armSubsystem.getAngle().lt(Arm.climbAngle))
        .andThen(
            armSubsystem.setAngleCommand(Arm.stowAngle)
            .alongWith(Commands.waitUntil(() -> armSubsystem.getAngle().gte(Arm.retractAngle)).andThen(
                pneumaticsSubsystem.retract(),
                Commands.waitSeconds(Pneumatics.retractionTime.in(Seconds))
            ))
        )
        .withName("Enter Full Stow");
    }

    public static Command leaveStow(ArmSubsystem armSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return armSubsystem.setAngleCommand(Arm.stowAngle)
        .andThen(
            pneumaticsSubsystem.extend(),
            Commands.waitSeconds(Pneumatics.extensionTime.in(Seconds))
        )
        .withName("Leave Full Stow");
    }
   
    public static Command groundIntake(ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return Commands.either(intakeSubsystem.intakeCommand(), leaveStow(armSubsystem, pneumaticsSubsystem)
        .andThen(
            armSubsystem.setAngleCommand(Arm.intakeAngle),
            armSubsystem.runOnce(armSubsystem::stop),
            intakeSubsystem.intakeCommand()
            .alongWith(Commands.waitUntil(intakeSubsystem::isSensorTripped).andThen(enterStow(armSubsystem, pneumaticsSubsystem)))
        ), intakeSubsystem::isSensorTripped)
        .withName("Intake Ground");
    }

    public static Command ampPosition(ArmSubsystem armSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return leaveStow(armSubsystem, pneumaticsSubsystem)
        .andThen(armSubsystem.setAngleCommand(Arm.ampAngle))
        .withName("Amp Position");
    }
 
    public static Command ampPositionAndCenter(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return ampPosition(armSubsystem, pneumaticsSubsystem)
        .andThen(driveSubsystem.trajectoryToPoseCommand(() -> {
            TrajectoryConfig config = new TrajectoryConfig(FeetPerSecond.of(6), FeetPerSecondSquared.of(5));
            Pose2d ampPose = (Robot.onRedAlliance()) ? Field.redAmpScorePos : Field.blueAmpScorePos;
            return driveSubsystem.generateTrajectory(ampPose, config);
        })).withName("Auto Amp Center");
    }

    public static Command scoreSpeakerBase(ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem, PneumaticsSubsystem pneumaticsSubsystem, boolean stowAtEnd) {
        Command endStowCommand = stowAtEnd ? enterStow(armSubsystem, pneumaticsSubsystem) : Commands.none();
        return leaveStow(armSubsystem, pneumaticsSubsystem)
        .andThen(
            armSubsystem.setAngleCommand(Arm.speakerBaseAngle)
                .alongWith(shooterSubsystem.spinUpFlywheelCommand()),
            shooterSubsystem.releaseNoteCommand(),
            endStowCommand
        )
        .withName("Score Speaker Base");
    }

    public static Command scoreSpeaker(Measure<Angle> angle, DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return leaveStow(armSubsystem, pneumaticsSubsystem)
        .andThen(
            armSubsystem.setAngleCommand(angle)
                .alongWith(shooterSubsystem.spinUpFlywheelCommand()),
            shooterSubsystem.releaseNoteCommand(),
            enterStow(armSubsystem, pneumaticsSubsystem)
        )
        .withName("Score Speaker");
    }

    public static Command turnToSpeaker(DriveSubsystem driveSubsystem) {
        return driveSubsystem.turnCommand(
            () -> {
                Translation2d speakerPos = (Robot.onRedAlliance()) ? Field.redSpeakerPosition : Field.blueSpeakerPosition;
                return Radians.of(driveSubsystem.getFieldPose().getTranslation().minus(speakerPos).getAngle().getRadians());
            }
        )
        .withTimeout(2)
        .withName("Turn to Speaker");
    }

    public static Command turnAndScoreSpeaker(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem shooterSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return turnToSpeaker(driveSubsystem)
        .alongWith(
            leaveStow(armSubsystem, pneumaticsSubsystem),
            shooterSubsystem.spinUpFlywheelCommand()
        )
        .andThen(
            armSubsystem.setAngleCommand(() -> {
                if (!driveSubsystem.hasInitalizedFieldPose()) return Arm.speakerBaseAngle;
                return Shooter.getPredictedAngle(Field.getDistanceToSpeaker(driveSubsystem.getFieldPose()));
            }),
            Commands.waitSeconds(0.5), // Wait for arm to settle
            shooterSubsystem.releaseNoteCommand(),
            // driveSubsystem.motionMagicVelocityCommand(FeetPerSecond.of(6), FeetPerSecond.of(6), FeetPerSecondSquared.of(9))
            // .andThen(Commands.waitSeconds(0.6)).finallyDo(driveSubsystem::stop)
            // .alongWith(enterStow(armSubsystem, pneumaticsSubsystem))
            enterStow(armSubsystem, pneumaticsSubsystem)
        )
        .withName("Aim and Score Speaker");
    }

    public static Command extendClimber(ArmSubsystem armSubsystem, ClimbSubsystem climbSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return Commands.either(Commands.none(), leaveStow(armSubsystem, pneumaticsSubsystem), () -> pneumaticsSubsystem.isExtended())
        .andThen(
            armSubsystem.setAngleCommand(Arm.climbAngle)
            .alongWith(climbSubsystem.setVoltage(Climb.climbPower))
        )
        .withName("Extend Climber");
    }

    public static Command retractClimber(ArmSubsystem armSubsystem, ClimbSubsystem climbSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        return Commands.either(Commands.none(), leaveStow(armSubsystem, pneumaticsSubsystem), () -> pneumaticsSubsystem.isExtended())
        .andThen(
            armSubsystem.setAngleCommand(Arm.climbAngle),
            climbSubsystem.setVoltage(Climb.climbPower.negate())
        )
        .withName("Retract Climber");
    }

    private static Trajectory latestTrajectory;
    public static Command extendAndCenterOnChain(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, ClimbSubsystem climbSubsystem, PneumaticsSubsystem pneumaticsSubsystem) {
        final Measure<Angle> climberDeadband = Rotations.of(3);
        final Measure<Distance> minDistanceForWaypoint = Feet.of(3);
        return extendClimber(armSubsystem, climbSubsystem, pneumaticsSubsystem)
        .alongWith(
            Commands.defer(() -> {
                List<Pose2d> points = new ArrayList<>();
                TrajectoryConfig config = new TrajectoryConfig(MetersPerSecond.of(3.5), MetersPerSecondPerSecond.of(1))
                    .setKinematics(Drive.diffKinematics)
                    .setReversed(true)
                    .addConstraint(new CentripetalAccelerationConstraint(0.5));
                Measure<Distance> waypointOffset = Feet.of(3.5);
                Pose2d fieldPose = driveSubsystem.getFieldPose(); 
                Pose2d endPose = Field.getNearestChain(fieldPose);
                Pose2d waypoint = endPose.plus(new Transform2d(waypointOffset, Meters.of(0), Rotation2d.fromDegrees(0)));
                if (fieldPose.relativeTo(waypoint).getX() > minDistanceForWaypoint.in(Meters)) { // Check if I'm far enough away to use my waypoint
                    points.add(waypoint);
                }
                points.add(endPose);
                latestTrajectory = driveSubsystem.generateTrajectory(points, config); 
                Command trajectoryCmd = driveSubsystem.trajectoryToPoseCommand(() -> {
                    return latestTrajectory;
                });
                Measure<Time> trajectoryTime = Seconds.of(latestTrajectory.getTotalTimeSeconds());
                double waitTime = MathUtil.clamp(Climb.timeToExtendClimbers.plus(Climb.timeToExtendClimbersMargin).minus(trajectoryTime).in(Seconds), 0, Double.MAX_VALUE);
                Command waitCommand = Commands.waitSeconds(waitTime); // Only wait required time to extend climbers
                return waitCommand
                .until(() -> Util.inRange(climbSubsystem.getPosition().in(Rotations) - Climb.softLimitSwitchConfig.ForwardSoftLimitThreshold, climberDeadband.in(Rotations))) 
                .andThen(trajectoryCmd);
            }, Set.of(driveSubsystem))
        )
        .withName("Go to Climb");
    }
}
