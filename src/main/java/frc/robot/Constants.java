package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Current;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

public class Constants {

    public static final double degreesPerRotation = 360;
    public static final double maxBatteryVoltage = 12;
    public static final Measure<Distance> robotLength = Inches.of(29 + 6); // Robot is 29 in. by 29 in. Bumper width is 3 in.
    public static final Measure<Distance> halfRobotLength = robotLength.divide(2);
    public static final Velocity<Velocity<Distance>> FeetPerSecondSquared = FeetPerSecond.per(Second);

    public static class Joysticks {
        // Ports
        public static final int joystickPort = 0;
    }
    
    public static class Drive {
        public static final int LID = 0;
        public static final int LFollowerID = 1;
        public static final int RID = 2;
        public static final int RFollowerID = 3;
        public static final int pigeonID = 9;
        public static final double gearRatio = 10.71;
        public static final Measure<Distance> wheelDiameter = Inches.of(6.125); 
        public static final Measure<Distance> trackWidth = Inches.of(24.5); 
        public static final DifferentialDriveKinematics diffKinematics = new DifferentialDriveKinematics(trackWidth); 
        public static final Slot0Configs velocityLeftConfig = new Slot0Configs().withKP(0.3).withKD(0).withKS(0.074181).withKV(0.10813).withKA(0.018);
        public static final Slot0Configs velocityRightConfig = new Slot0Configs().withKP(0.3).withKD(0).withKS(0.15188).withKV(0.10875).withKA(0.019);
        // RoboRIO PID auto turning
        public static final SlotConfigs turnPIDConfig = new SlotConfigs().withKP(0.1).withKD(0.009);
        public static final Measure<Angle> turnDeadband = Degrees.of(2);
        public static final Measure<Voltage> minTurnPIDVoltage = Volts.of(0.4); 
        public static final Measure<Voltage> maxTurnPIDVoltage = Volts.of(4); 
        // Trajectories
        public static final double ramseteB = 2.0;
        public static final double ramseteZeta = 0.7;
    }

    public static class Driving {
        public static final double speedAdjustment = 1;
        public static final double turnAdjustment = 0.5;
        public static final double trimAdjustment = 0.25;
        public static final double trimFadeout = 0.25;
        public static final boolean squareInputs = true;
        public static final boolean constantCurvature = true;
        public static final double joystickDeadband = 0.07;
        public static final double deadbandSmoothing = 1.5;
    }

    public static class Arm {
        public static final int ID = 4;
        public static final int followerID = 5;
        public static final int throughBoreEncoderID = 0;
        public static final double gearRatio = 100;
        public static final Slot0Configs motionMagicConfig = new Slot0Configs().withKP(500).withKD(0);
        public static final FeedbackConfigs feedbackConfig = new FeedbackConfigs().withSensorToMechanismRatio(gearRatio);
        public static final Measure<Time> calibrateTime = Seconds.of(0.25);
        public static final Measure<Angle> encoderOffset = Degrees.of(54); 
        public static final Measure<Angle> boreEncoderHardstop = Degrees.of(85);
        public static final Measure<Angle> angleDeadband = Degrees.of(0.75); // Unfortunately this affects the shooter map, so don't change it
        public static final Measure<Angle> startAngle = Degrees.of(16); 
        public static final Measure<Angle> intakeAngle = Degrees.of(18); 
        public static final Measure<Angle> climbAngle = Degrees.of(28); // Just hovering off the ground
        public static final Measure<Angle> speakerBaseAngle = Degrees.of(42);
        public static final Measure<Angle> ampAngle = Degrees.of(76.0); 
        public static final Measure<Angle> retractAngle = Degrees.of(45); // Safe to retract pistons
        public static final Measure<Angle> stowAngle = Degrees.of(63);
        public static final Measure<Angle> minAngle = Degrees.of(18);
        public static final Measure<Angle> maxAngle = Degrees.of(80);
        public static final SoftwareLimitSwitchConfigs softLimitSwitchConfig = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true).withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(maxAngle.in(Rotations)).withReverseSoftLimitThreshold(minAngle.in(Rotations));
        public static final double defaultVelocity = 100;
        public static final double defaultAcceleration = 100;
        public static final double defaultJerk = 0;
    }

    public static class Pneumatics {
        public static final int leftBottomChannel = 0;
        public static final int rightBottomChannel = 2;
        public static final Measure<Time> extensionTime = Seconds.of(0);
        public static final Measure<Time> retractionTime = Seconds.of(0.5);
    }

    public static class Intake {
        public static final int intakeID = 6;
        public static final int timeOfFlightID = 0;
        public static final double intakeSpeed = 1; 
        public static final Measure<Distance> noteDetectionRange = Millimeters.of(130); 
        public static final Measure<Time> waitAfterTrip = Seconds.of(0);
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
            interpolatingShootingMap.put(117.0, 50.5);
            interpolatingShootingMap.put(112.7, 50.5);
            interpolatingShootingMap.put(102.1, 50.5);
            interpolatingShootingMap.put(95.0, 50.5); 
            interpolatingShootingMap.put(90.5, 50.5); // Map testing ends here
            interpolatingShootingMap.put(86.0, 50.5);
            interpolatingShootingMap.put(80.5, 49.5);
            interpolatingShootingMap.put(76.3, 49.0);
            interpolatingShootingMap.put(71.1, 49.0);
            interpolatingShootingMap.put(61.6, 44.5);
            interpolatingShootingMap.put(52.6, Arm.speakerBaseAngle.in(Degrees));
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
        public static final Measure<Angle> maxPosition = Rotations.of(330);
        public static final Measure<Time> timeToExtendClimbers = Seconds.of(3);
        public static final Measure<Time> timeToExtendClimbersMargin = Seconds.of(1);
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
        public static final List<Integer> stageTags = List.of(11, 12, 13, 14, 15, 16);
        public static final Measure<Angle> maxYaw = Degrees.of(30);
    }

    public class LED {
        public static final int ledPort = 0;
    }

}
