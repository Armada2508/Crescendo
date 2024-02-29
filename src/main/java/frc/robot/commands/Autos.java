package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static frc.robot.commands.Routines.groundIntake;
import static frc.robot.commands.Routines.scoreSpeakerBase;
import static frc.robot.commands.Routines.stowCommand;
import static frc.robot.commands.Routines.turnAndScoreSpeaker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class Autos { //! There's a lot of magic numbers in these Autos

    private Autos() {}

    /**
     * Leaves the ROBOT STARTING ZONE.
     */
    public static Command leaveStartingZone(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem) {
        return stowCommand(armSubsystem) 
        .andThen(driveSubsystem.driveDistanceVelCommand(Feet.of(5), FeetPerSecond.of(2.5)));
    }

    /**
     * Scores a preloaded NOTE at the SUBWOOFER and leaves the ROBOT STARTING ZONE.
     */
    public static Command scoreSpeakerBaseAndLeave(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return stowCommand(armSubsystem)
        .andThen(Commands.waitSeconds(1))
        .andThen(scoreSpeakerBase(armSubsystem, intakeShooterSubsystem))
        .andThen(driveSubsystem.driveDistanceVelCommand(Feet.of(5), FeetPerSecond.of(2.5)));
    }
    
    /**
     * Scores a preloaded NOTE into the SPEAKER using vision, faces the ALLIANCE WALL, and leaves the ROBOT STARTING ZONE.
     */
    public static Command scoreSpeakerAndLeave(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return stowCommand(armSubsystem)
        .andThen(Commands.waitSeconds(1))
        .andThen(turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem))
        .andThen(faceShooterTowardsWall(driveSubsystem))
        .andThen(driveSubsystem.driveDistanceVelCommand(Feet.of(5), FeetPerSecond.of(2.5))); 
    }

    /**
     * Scores a preloaded NOTE into the SPEAKER using vision, faces the ALLIANCE WALL, intakes the NOTE behind it and scores it into the SPEAKER.
     */
    public static Command scoreSpeakerTwiceBehind(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, IntakeShooterSubsystem intakeShooterSubsystem) {
        return stowCommand(armSubsystem)
        .andThen(Commands.waitSeconds(1))
        .andThen(Commands.either(
            turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem)
            .andThen(
            Commands.waitSeconds(2) // Wait for flywheels to spin down
                .alongWith(faceShooterTowardsWall(driveSubsystem))
            )
            .andThen(
                intakeShooterSubsystem.brakeShooter(),
                driveSubsystem.setVelocityCommand(FeetPerSecond.of(4), FeetPerSecond.of(4)),
                groundIntake(armSubsystem, intakeShooterSubsystem)
                    .alongWith(Commands.waitUntil(intakeShooterSubsystem::isSensorTripped).finallyDo(driveSubsystem::stop))
            )
            .andThen(
                driveSubsystem.setVelocityCommand(FeetPerSecond.of(-3), FeetPerSecond.of(-3)),
                Commands.waitUntil(() -> driveSubsystem.getDistanceToSpeaker().lte(Inches.of(92))).finallyDo(driveSubsystem::stop),
                Commands.waitSeconds(1),
                turnAndScoreSpeaker(driveSubsystem, armSubsystem, intakeShooterSubsystem)
            ), 
        Commands.none(), driveSubsystem::hasInitalizedFieldPose));
        
    }

    private static Command faceShooterTowardsWall(DriveSubsystem driveSubsystem) {
        return driveSubsystem.turnCommand(
            () -> {
                if (!driveSubsystem.hasInitalizedFieldPose()) driveSubsystem.getFieldAngle();
                return (Robot.onRedAlliance()) ? Degrees.of(180) : Degrees.of(0);
            }
        );
    }

}
