package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.lib.drive.DriveCommand.DriveConfig;

public class Constants {

    public static final double degreesPerRotation = 360;
    public static final Measure<Distance> robotLength = Inches.of(29 + 6); // Robot is 29 in. by 29 in. Bumper width is 3 in.
    public static final Measure<Distance> halfRobotLength = robotLength.divide(2);
    public static final Velocity<Velocity<Distance>> FeetPerSecondSquared = FeetPerSecond.per(Second);

    public static class Joysticks {
        // Ports
        public static final int joystickPort = 0;
        public static final int buttonBoardPort = 1;
    }
    
    public static class Drive {
        public static final int LID = 0;
        public static final int LFollowerID = 1;
        public static final int RID = 2;
        public static final int RFollowerID = 3;
        public static final int pigeonID = 9;
        public static final DriveConfig joystickDriveConfig = new DriveConfig(
            1, 0.5, 0.25, 0.2, true, true, 1.5, 0.07 // Speed Multi, Turn Multi, Trim Multi, Slow Speed, Square Inputs, Constant Curvature, Slew Rate, Joystick Deadband
        );
        public static final double trimFadeout = 0.25;
        public static final double gearRatio = 10.71;
        public static final Measure<Distance> wheelDiameter = Inches.of(6.125); 
        public static final Measure<Distance> trackWidth = Inches.of(24.5); 
        public static final DifferentialDriveKinematics diffKinematics = new DifferentialDriveKinematics(trackWidth); 
        public static final Slot0Configs velocityLeftConfig = new Slot0Configs().withKP(0.3).withKD(0).withKS(0.074181).withKV(0.10813).withKA(0.018);
        public static final Slot0Configs velocityRightConfig = new Slot0Configs().withKP(0.3).withKD(0).withKS(0.15188).withKV(0.10875).withKA(0.019);
        // RoboRIO PID auto turning
        public static final SlotConfigs turnPIDConfig = new SlotConfigs().withKP(0.1).withKD(0.009);
        public static final Measure<Angle> turnDeadband = Degrees.of(2);
        public static final Measure<Voltage> minTurnPIDVoltage = Volts.of(0.3); 
        public static final Measure<Voltage> maxTurnPIDVoltage = Volts.of(4); 
        // Trajectories
        public static final double ramseteB = 2.0;
        public static final double ramseteZeta = 0.7;
        public static final TrajectoryConfig trajectoryConfig = 
            new TrajectoryConfig(MetersPerSecond.of(2.0), MetersPerSecondPerSecond.of(1.0)).setKinematics(diffKinematics).addConstraint(new CentripetalAccelerationConstraint(1));
    }

    public static class Arm {
        public static final int ID = 4;
        public static final int followerID = 5;
        public static final int throughBoreEncoderID = 0;
        public static final double gearRatio = 100;
        public static final double gravityFeedforward = 0; //! Find this
        public static final Slot0Configs motionMagicConfig = new Slot0Configs().withKP(500).withKD(0);
        public static final FeedbackConfigs feedbackConfig = new FeedbackConfigs().withSensorToMechanismRatio(gearRatio);
        public static final Measure<Time> calibrateTime = Seconds.of(0.25);
        public static final Measure<Angle> encoderOffset = Degrees.of(56);
        public static final Measure<Angle> boreEncoderHardstop = Degrees.of(85);
        public static final Measure<Angle> angleDeadband = Degrees.of(0.75); // Unfortunately this affects the shooter map, so don't change it
        public static final Measure<Angle> startAngle = Degrees.of(16); 
        public static final Measure<Angle> intakeAngle = Degrees.of(18); 
        public static final Measure<Angle> stowAngle = Degrees.of(28); 
        public static final Measure<Angle> speakerBaseAngle = Degrees.of(39);
        public static final Measure<Angle> ampAngle = Degrees.of(75.0); 
        public static final Measure<Angle> minAngle = Degrees.of(18);
        public static final Measure<Angle> maxAngle = Degrees.of(80);
        public static final SoftwareLimitSwitchConfigs softLimitSwitchConfig = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true).withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(maxAngle.in(Rotations)).withReverseSoftLimitThreshold(minAngle.in(Rotations));
        public static final double defaultVelocity = 100;
        public static final double defaultAcceleration = 100;
        public static final double defaultJerk = 0;
    }

    public class Wrist {
        public static final int ID = 13;
        public static final int followID = 14;
        public static final int boreEncoderID = 1;
        public static final double gearRatio = 50;
        public static final Measure<Angle> stowAngle = Degrees.of(0); //! Tune this value
        public static final Measure<Angle> intakeShootAngle = Degrees.of(0); //! Tune this value
        public static final Measure<Angle> maxAngle = Degrees.of(0); //! Tune this value
        public static final Measure<Angle> minAngle = Degrees.of(0); //! Tune this value
        public static final Measure<Angle> angleDeadband = Degrees.of(0); //! Tune this value
        public static final Measure<Angle> encoderOffset = Degrees.of(0); //! Tune this value (if needed)
        public static final SoftwareLimitSwitchConfigs softLimitSwitchConfig = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true).withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(maxAngle.in(Rotations)).withReverseSoftLimitThreshold(minAngle.in(Rotations));
        public static final double defaultVelocity = 100;
        public static final double defaultAcceleration = 100;
        public static final double defaultJerk = 0;
        public static final Slot0Configs motionMagicConfig = new Slot0Configs().withKP(0).withKD(0); //! Find this
        public static final FeedbackConfigs feedbackConfig = new FeedbackConfigs().withSensorToMechanismRatio(gearRatio);
    }

    public static class Intake {
        public static final int intakeID = 6;
        public static final int timeOfFlightID = 0;
        public static final double intakeSpeed = 1; 
        public static final Measure<Distance> noteDetectionRange = Millimeters.of(130); 
        public static final Measure<Time> waitAfterTrip = Seconds.of(0.25);
        public static final Measure<Time> noteSettleTime = Seconds.of(0.25);
        public static final Measure<Time> backOffNoteTime = Seconds.of(0.05);
        public static final double backOffSpeed = -0.3;
    }

    public static class Shooter {
        public static final int shooterID = 7;
        public static final int shooterFollowerID = 8;
        public static final double indexSpeed = 1;
        public static final Measure<Time> flywheelChargeTime = Seconds.of(0.5);
        public static final Measure<Voltage> speakerShootPower = Volts.of(9); 
        public static final Measure<Time> speakerTimeToShoot = Seconds.of(0.3);
        public static final Measure<Voltage> ampShootPower = Volts.of(-12); 
        public static final Measure<Time> ampTimeToShoot = Seconds.of(1.0); 
        public static final Measure<Velocity<Angle>> minShooterVelocityBraking = Rotations.per(Minute).of(3000);
        // Higher arm angle = lower note height
        /**INPUT: Distance (in.), OUTPUT: Angle (deg.) */ 
        private static final InterpolatingDoubleTreeMap interpolatingShootingMap = new InterpolatingDoubleTreeMap(); 
        public static final Measure<Distance> maxShootDistance = Inches.of(90.0);
        static {
            interpolatingShootingMap.put(117.0, 50.0);
            interpolatingShootingMap.put(112.7, 49.5);
            interpolatingShootingMap.put(102.1, 48.5);
            interpolatingShootingMap.put(95.0, 48.0);
            interpolatingShootingMap.put(90.5, 47.75);
            interpolatingShootingMap.put(85.4, 47.50);
            interpolatingShootingMap.put(81.9, 47.0);
            interpolatingShootingMap.put(76.3, 45.0);
            interpolatingShootingMap.put(70.9, 44.0);
            interpolatingShootingMap.put(61.6, 42.0);
            interpolatingShootingMap.put(52.6, 39.0);
        }
        public static Measure<Angle> getPredictedAngle(Measure<Distance> distance) {
            return Degrees.of(interpolatingShootingMap.get(distance.in(Inches)));
        }
    }

    public static class Climb { 
        public static final int climbID = 10;
        public static final int climbFollowID = 11;
        public static final Measure<Voltage> zeroVoltage = Volts.of(-0.5);
        public static final Measure<Current> zeroTripTorqueCurrent = Amps.of(2);
        public static final Measure<Voltage> climbPower = Volts.of(12);
        public static final Measure<Angle> minPosition = Rotations.of(0);
        public static final Measure<Angle> maxPosition = Rotations.of(295);
        public static final SoftwareLimitSwitchConfigs softLimitSwitchConfig = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true).withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(maxPosition.in(Rotations)).withReverseSoftLimitThreshold(minPosition.in(Rotations));
    }

    public static class Vision {
        public static final String tagCameraName = "OV9281";
        public static final String noteCameraName = "LifeCam";
        public static final Transform3d robotToCamera = new Transform3d(Units.inchesToMeters(-11.5), Units.inchesToMeters(5.5), 0, new Rotation3d(0, Units.degreesToRadians(-30), Units.degreesToRadians(180)));
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        public static final Measure<Distance> maxAvgTagDistance = Feet.of(13);
        // Meters, Meters, Radians
        public static final Matrix<N3, N1> singleTagVisionStdDevs = VecBuilder.fill(4, 4, 8); 
        public static final Matrix<N3, N1> multiTagVisionStdDevs = VecBuilder.fill(0.5, 0.5, 1); 
        public static final Matrix<N3, N1> untrustedVisionStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    }

    public static class Field {
        // Origin is always the blue alliance origin
        public static final Pose2d origin = new Pose2d();
        public static final Field2d simulatedField = new Field2d();
        public static final Measure<Distance> fieldLength = Inches.of(651.25);
        public static final Measure<Distance> fieldWidth = Inches.of(323.25); // Also kind of like the height of the field
        public static final Measure<Angle> minAngle = Degrees.of(-180);
        public static final Measure<Angle> maxAngle = Degrees.of(180);
        public static final Measure<Distance> subWooferLength = Inches.of(36.125);
        public static final Translation2d blueSpeakerPosition = new Translation2d(Meters.of(0), Meters.of(5.55));
        public static final Translation2d redSpeakerPosition = new Translation2d(Field.fieldLength, Meters.of(5.55));    
        // Position of the front of the SUBWOOFER, facing the SPEAKER
        public static final Pose2d blueSpeakerBaseScorePos = new Pose2d(subWooferLength.plus(halfRobotLength).plus(Inches.of(4)), Meters.of(5.55), Rotation2d.fromDegrees(0));    
        public static final Pose2d redSpeakerBaseScorePos = new Pose2d(Field.fieldLength.minus(subWooferLength).minus(halfRobotLength).minus(Inches.of(4)), Meters.of(5.55), Rotation2d.fromDegrees(180));  
        // Position in front of the AMP, facing the AMP  
        public static final Pose2d blueAmpScorePos = new Pose2d(Inches.of(72.5), fieldWidth.minus(halfRobotLength), Rotation2d.fromDegrees(90));
        public static final Pose2d redAmpScorePos = new Pose2d(fieldLength.minus(Inches.of(72.5)), fieldWidth.minus(halfRobotLength), Rotation2d.fromDegrees(90));
        // Blue Notes, plus fudge for AUTO
        public static final Translation2d blueTopNotePos = new Translation2d(Inches.of(114), Inches.of(275.625 + 8)); 
        public static final Translation2d blueMidNotePos = new Translation2d(Inches.of(114), Inches.of(218.625)); 
        public static final Translation2d blueLowNotePos = new Translation2d(Inches.of(114), Inches.of(161.625)); 
        // Red Notes, plus fudge for AUTO
        public static final Translation2d redTopNotePos = new Translation2d(Field.fieldLength.minus(Inches.of(114)), Inches.of(275.625 + 8)); 
        public static final Translation2d redMidNotePos = new Translation2d(Field.fieldLength.minus(Inches.of(114)), Inches.of(218.625)); 
        public static final Translation2d redLowNotePos = new Translation2d(Field.fieldLength.minus(Inches.of(114)), Inches.of(161.625)); 
        public enum Note {
            TOP,
            MID,
            LOW
        }
        public static Translation2d getNote(Note note) {
            return switch (note) {
                case TOP -> Robot.onRedAlliance() ? Field.redTopNotePos : Field.blueTopNotePos;
                case MID -> Robot.onRedAlliance() ? Field.redMidNotePos : Field.blueMidNotePos;
                case LOW -> Robot.onRedAlliance() ? Field.redLowNotePos : Field.blueLowNotePos;
                default -> throw new IllegalArgumentException("Illegal Note: " + note);
            };
        }
        // Blue Stage April Tag Poses
        public static final Pose2d blueStageCenter = new Pose2d(Meters.of(5.32), Meters.of(4.11), Rotation2d.fromDegrees(0));
        public static final Pose2d blueStageLeft = new Pose2d(Meters.of(4.64), Meters.of(4.5), Rotation2d.fromDegrees(120));
        public static final Pose2d blueStageRight = new Pose2d(Meters.of(4.64), Meters.of(3.71), Rotation2d.fromDegrees(-120));
        public static final Measure<Distance> chainToStage = Inches.of(16.625);
        public static final Measure<Distance> chainClimbOffset = chainToStage.plus(Inches.of(-4));

        public static final Pose2d blueChainCenter = new Pose2d(Meters.of(5.32).plus(chainClimbOffset), Meters.of(4.11), blueStageCenter.getRotation());
        public static final Translation2d leftChainVector = new Translation2d(chainClimbOffset, Meters.of(0)).rotateBy(blueStageLeft.getRotation());
        public static final Pose2d blueChainLeft = new Pose2d(blueStageLeft.getTranslation().plus(leftChainVector), blueStageLeft.getRotation());
        public static final Translation2d rightChainVector = new Translation2d(chainClimbOffset, Meters.of(0)).rotateBy(blueStageRight.getRotation());
        public static final Pose2d blueChainRight = new Pose2d(blueStageRight.getTranslation().plus(rightChainVector), blueStageRight.getRotation());
        public static final Pose2d redChainCenter = new Pose2d(Field.fieldLength.in(Meters) - blueChainCenter.getX(), blueChainCenter.getY(), Rotation2d.fromDegrees(180));
        public static final Pose2d redChainLeft = new Pose2d(Field.fieldLength.in(Meters) - blueChainRight.getX(), blueChainRight.getY(), Rotation2d.fromDegrees(-60));
        public static final Pose2d redChainRight = new Pose2d(Field.fieldLength.in(Meters) - blueChainLeft.getX(), blueChainLeft.getY(), Rotation2d.fromDegrees(60));
 
        public static Pose2d getNearestChain(Pose2d pose) {
            Pose2d chainCenter = blueChainCenter;
            Pose2d chainLeft = blueChainLeft;
            Pose2d chainRight = blueChainRight;
            if (Robot.onRedAlliance()) {
                chainCenter = redChainCenter;
                chainLeft = redChainLeft;
                chainRight = redChainRight;
            }
            double distanceCenter = pose.getTranslation().getDistance(chainCenter.getTranslation());
            double distanceLeft = pose.getTranslation().getDistance(chainLeft.getTranslation());
            double distanceRight = pose.getTranslation().getDistance(chainRight.getTranslation());
            double distance = Math.min(Math.min(distanceLeft, distanceRight), distanceCenter);
            if (distance == distanceCenter) {
                return chainCenter;
            } 
            else if (distance == distanceLeft) {
                return chainLeft;
            }
            else {
                return chainRight;
            }
        }

        public static Measure<Distance> getDistanceToSpeaker(Pose2d pose) {
            Translation2d speakerPos = (Robot.onRedAlliance()) ? Field.redSpeakerPosition : Field.blueSpeakerPosition;
            return Meters.of(pose.getTranslation().getDistance(speakerPos));
        }
    }

    public class LED {
        public static final int ledPort = 12;
    }
}
