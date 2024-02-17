package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.lib.drive.DriveCommand.DriveConfig;

public class Constants {

    public static final double gravity = 9.81;
    public static final double degreesPerRotation = 360;
    public static final Measure<Distance> robotLength = Inches.of(29 + 6); // Bumper width is 3 in.
    public static final Measure<Distance> halfRobotLength = robotLength.divide(2);

    public static class Joysticks {
        // Ports
        public static final int joystickPort = 0;
        public static final int buttonBoardPort = 1;
        // Joystick
        public static final int driveSlowButton = 12;
        // Button Board
        public static final int driveReverseButton = 9;
        public static final int solenoidButton = 10;
    }
    
    public static class Drive {
        public static final int LID = 0;
        public static final int LFollowID = 1;
        public static final int RID = 2;
        public static final int RFollowID = 3;
        public static final int pigeonID = 8;
        public static final DriveConfig joystickDriveConfig = new DriveConfig(
            0.25, 0.5, 0.1, 0.2, true, true, 1.5, 0.07 // Speed Multi, Turn Multi, Trim Multi, Slow Speed, Square Inputs, Constant Curvature, Slew Rate, Joystick Deadband
        );
        public static final double gearRatio = 10.71;
        public static final Measure<Distance> wheelDiameter = Inches.of(6.125); 
        public static final Measure<Distance> trackWidth = Inches.of(24.5); 
        public static final DifferentialDriveKinematics diffKinematics = new DifferentialDriveKinematics(trackWidth); 
        public static final Slot0Configs velocityLeftConfig = new Slot0Configs().withKP(0.3).withKD(0).withKS(0.18).withKV(0.11);
        public static final Slot0Configs velocityRightConfig = new Slot0Configs().withKP(0.3).withKD(0).withKS(0.23).withKV(0.11);
        public static final Slot1Configs motionMagicConfig = new Slot1Configs().withKP(0).withKD(0).withKS(0).withKV(0); //! Have to tune for these values
        public static final int motionMagicSlot = 1;
        public static final Measure<Distance> driveDeadband = Inches.of(0.1); //! Tune this
        // RoboRIO PID auto turning
        public static final SlotConfigs turnPIDConfig = new SlotConfigs().withKP(0.03).withKD(0);
        public static final Measure<Angle> turnDeadband = Degrees.of(0.5);
        public static final Measure<Voltage> maxTurnPIDVoltage = Volts.of(3); 
        // Trajectories
        public static final double ramseteB = 2.0;
        public static final double ramseteZeta = 0.7;
        public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(MetersPerSecond.of(1.0), MetersPerSecondPerSecond.of(0.5)).setKinematics(diffKinematics);
    }

    public static class Arm {
        public static final int ID = 4;
        public static final int followID = 5;
        public static final int relayChannel = 0;
        public static final double gearRatio = 100;
        public static final double gravityFeedforward = 0; //! Find this
        public static final Slot0Configs motionMagicConfig = new Slot0Configs().withKP(500).withKD(0); //! Have to tune for these values
        public static final FeedbackConfigs feedbackConfig = new FeedbackConfigs().withSensorToMechanismRatio(gearRatio);
        public static final Measure<Angle> angleDeadband = Degrees.of(0.5); //! Tune this
        public static final Measure<Angle> startAngle = Degrees.of(90); //! Find angle at startup
        public static final Measure<Angle> solenoidAngle = Degrees.of(100); //! Find angle of solenoid
        public static final Measure<Angle> solenoidBounds = Degrees.of(20); //! Tune this
        public static final Measure<Angle> stowAngle = Degrees.of(0); //! Find angle for stow
        public static final Measure<Angle> pickupAngle = Degrees.of(0); //! Find angle for pickup
        public static final Measure<Angle> ampAngle = Degrees.of(0); //! Find angle for amp
        public static final Measure<Angle> speakerAngle = Degrees.of(36);
        public static final Measure<Angle> minAngle = Degrees.of(0);
        public static final Measure<Angle> maxAngle = Degrees.of(110);
    }

    public static class Intake {
        public static final int intakeID = 6;
        public static final int timeOfFlightID = 0;
        public static final Measure<Distance> noteDetectionRange = Millimeters.of(10); //! Tune this
        public static final double intakeSpeed = 0.5; //! Tune this
        public static final Measure<Time> waitTimeAfterTrip = Seconds.of(1); //! Tune this
    }

    public static class Shooter {
        public static final int shooterID = 7;
        public static final int shooterFollowID = 8;
        public static final double indexSpeed = 0.5; //! Tune this
        public static final Measure<Time> flywheelChargeTime = Seconds.of(1.5); //! Tune this
        public static final Measure<Time> timeToShoot = Seconds.of(1); //! Tune this
        public static final Measure<Voltage> speakerShootPower = Volts.of(8);
        public static final Measure<Voltage> ampShootPower = Volts.of(2); //! Tune this
    }

    public static class Vision {
        public static final String tagCameraName = "OV9281";
        public static final String noteCameraName = "LifeCam";
        public static final Transform3d robotToCamera = new Transform3d(Units.inchesToMeters(-13.5), 0, 0, new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(180)));
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        public static final Matrix<N3, N1> poseStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class Field {
        // Origin is always the blue alliance origin
        public static final Pose2d origin = new Pose2d();
        public static final Field2d simulatedField = new Field2d();
        public static final Measure<Distance> fieldLength = Inches.of(651.25);
        public static final Measure<Distance> fieldWidth = Inches.of(323.25);
        private static final Measure<Distance> subWooferLength = Inches.of(36.125);
        public static final Translation2d blueSpeakerPosition = new Translation2d(Meters.of(0), Meters.of(5.55));
        public static final Translation2d redSpeakerPosition = new Translation2d(Field.fieldLength, Meters.of(5.55));    
        // Position of the front of the base of the speaker, facing the speaker
        public static final Pose2d blueSpeakerBaseScorePos = new Pose2d(subWooferLength.plus(halfRobotLength).plus(Inches.of(4)), Meters.of(5.55), Rotation2d.fromDegrees(0));    
        public static final Pose2d redSpeakerBaseScorePos = new Pose2d(Field.fieldLength.minus(subWooferLength).minus(halfRobotLength).minus(Inches.of(4)), Meters.of(5.55), Rotation2d.fromDegrees(180));    
        // All note positions are notes for the BLUE alliance
        public static final Translation2d lowerNoteCoordinate = new Translation2d(Inches.of(114), Inches.of(161.625)); 
        public static final Translation2d mediumNoteCoordinate = new Translation2d(Inches.of(114), Inches.of(218.625)); 
        public static final Translation2d upperNoteCoordinate = new Translation2d(Inches.of(114), Inches.of(275.625)); 
        public static final Pose2d lowerNotePickupLocation = new Pose2d(lowerNoteCoordinate.getX(), lowerNoteCoordinate.getY(), Rotation2d.fromDegrees(0));
        public static final Pose2d mediumNotePickupLocation = new Pose2d(mediumNoteCoordinate.getX(), mediumNoteCoordinate.getY(), Rotation2d.fromDegrees(0));
        public static final Pose2d upperNotePickupLocation = new Pose2d(upperNoteCoordinate.getX(), upperNoteCoordinate.getY(), Rotation2d.fromDegrees(0));
    }

}
