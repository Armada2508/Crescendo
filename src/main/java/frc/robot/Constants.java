package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.lib.drive.DriveCommand.DriveConfig;

public class Constants {

    public static final double gravity = 9.81;
    public static final double degreesPerRotation = 360;

    public static class Joysticks {
        // Ports
        public static final int joystickPort = 0;
        public static final int buttonBoardPort = 1;
        // Joystick
        public static final int driveSlowButton = 12;
        // Button Board
        public static final int driveReverseButton = 1;
    }
    
    public static class Drive {
        public static final int LID = 0;
        public static final int LFollowID = 1;
        public static final int RID = 2;
        public static final int RFollowID = 3;
        public static final int pigeonID = 8;
        public static final DriveConfig joystickDriveConfig = new DriveConfig(
            1, 0.5, 0.25, 0.2, true, 1.5, 0.07 // Speed Multi, Turn Multi, Trim Multi, Slow Speed, Square Inputs, Slew Rate, Joystick Deadband
        );
        public static final double gearRatio = 10.71;
        public static final Measure<Distance> wheelDiameter = Inches.of(6); 
        public static final Measure<Distance> trackWidth = Inches.of(24.5); //! Find this
        public static final DifferentialDriveKinematics diffKinematics = new DifferentialDriveKinematics(trackWidth); 
        public static final Slot0Configs motionMagicConfig = new Slot0Configs().withKP(0).withKD(0); //! Have to tune for these values
        public static final Slot1Configs velocityConfig = new Slot1Configs().withKP(0).withKD(0).withKV(0); //! Have to tune for these values
        public static final int velocitySlot = 1;
        public static final Measure<Distance> driveDeadband = Inches.of(0.5); //! Tune this
        // RoboRIO PID auto turning
        public static final SlotConfigs turnPIDConfig = new SlotConfigs().withKP(0).withKD(0); //! Have to tune for these values
        public static final Measure<Angle> turnDeadband = Degrees.of(0.5); //! Tune this
        public static final double maxTurnPIDSpeed = 0.25; //! Tune this
        // Trajectories
        public static final double ramseteB = 2.0;
        public static final double ramseteZeta = 0.7;
        public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(MetersPerSecond.of(0.75), MetersPerSecondPerSecond.of(0.5)).setKinematics(diffKinematics);
    }

    public static class Arm {
        public static final int ID = 4;
        public static final int followID = 5;
        public static final int throughBoreEncoderID = 0;
        public static final double gearRatio = 100;
        public static final double gravityFeedforward = 0; //! Find this
        public static final Slot0Configs motionMagicConfig = new Slot0Configs().withKP(0).withKD(0); //! Have to tune for these values
        public static final Measure<Angle> angleDeadband = Degrees.of(0.5); //! Tune this
        public static final double boreEncoderTicksPerRotation = 1024;
        public static final Measure<Angle> boreEncoderOffset = Degrees.of(0); //! Have to find this value
        public static final Measure<Angle> stowAngle = Degrees.of(0); //! Find angle for stow
        public static final Measure<Angle> pickupAngle = Degrees.of(0); //! Find angle for pickup
        public static final Measure<Angle> ampAngle = Degrees.of(0); //! Find angle for amp
        public static final Measure<Angle> speakerAngle = Degrees.of(0); //! Find angle for base speaker
        public static final Measure<Angle> minAngle = Degrees.of(-10);
        public static final Measure<Angle> maxAngle = Degrees.of(90);
    }

    public static class Intake {
        public static final int intakeID = 6;
        public static final int timeOfFlightID = 0;
        public static final Measure<Distance> noteDetectionRange = Millimeters.of(10); //! Tune this
        public static final double intakeSpeed = 0.5; //! Tune this
        public static final Measure<Time> waitTimeAfterTrip = Seconds.of(1); //! Tune this
    }

    public static class Shooter {
        public static final int shooterID = 8;
        public static final double indexSpeed = 0.5; //! Tune this
        public static final Measure<Distance> flywheelDiameter = Inches.of(4); //! Find this
        public static final Slot0Configs velocityConfig = new Slot0Configs().withKP(0).withKV(0).withKS(0); //! Have to tune for these values
        public static final Measure<Time> flywheelVelocityTimeout = Seconds.of(3);
        public static final Measure<Velocity<Angle>> velocityDeadband = RotationsPerSecond.of(1); //! Tune this
        public static final Measure<Time> timeToShoot = Seconds.of(1); //! Tune this
        public static final Measure<Velocity<Angle>> speakerShootSpeed = RotationsPerSecond.of(0); //! Tune this
        public static final Measure<Velocity<Angle>> ampShootSpeed = RotationsPerSecond.of(0); //! Tune this
    }

    public static class Vision {
        public static final String cameraName = "OV9281";
        public static final Transform3d robotToCamera = new Transform3d(); //! Find this
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        // public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        // public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
        public static final Matrix<N3, N1> poseStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class Field {
        public static final Pose2d origin = new Pose2d();
        public static final Field2d simulatedField = new Field2d();
        public static final Measure<Distance> fieldLength = Inches.of(651.25);
        public static final Measure<Distance> fieldWidth = Inches.of(323.25);
        public static final Measure<Distance> lowSpeakerHeight = Inches.of(78);
        public static final Translation2d speakerPos = new Translation2d(Meters.of(0), Meters.of(5.55));        
        public static final Translation2d lowerNoteCordinate = new Translation2d(Inches.of(114), Inches.of(161.625)); //note positions are based off of the blue cordinates
        public static final Translation2d mediumNoteCordinate = new Translation2d(Inches.of(114), Inches.of(218.625)); 
        public static final Translation2d upperNoteCordinate = new Translation2d(Inches.of(114), Inches.of(275.625)); 
        public static final Pose2d lowerNotePickupLocation = new Pose2d(lowerNoteCordinate.getX(), lowerNoteCordinate.getY(), Rotation2d.fromDegrees(0));
        public static final Pose2d mediumNotePickupLocation = new Pose2d(mediumNoteCordinate.getX(), mediumNoteCordinate.getY(), Rotation2d.fromDegrees(0));
        public static final Pose2d upperNotePickupLocation = new Pose2d(upperNoteCordinate.getX(), upperNoteCordinate.getY(), Rotation2d.fromDegrees(0));

        
    }
}
