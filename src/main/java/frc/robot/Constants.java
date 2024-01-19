package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SlotConfigs;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.drive.DriveCommand.DriveConfig;

public class Constants {

    public static final double GRAVITY = 9.81;
    
    public static class Drive {
        public static final int LID = 0;
        public static final int LFID = 1;
        public static final int RID = 2;
        public static final int RFID = 3;
        public static final int pigeonID = 9;
        public static final DriveConfig joystickDriveConfig = new DriveConfig(
            1, 1, 1, 0.5, true, 1.5, 0.07 // Speed Multi, Turn Multi, Trim Multi, Slow Speed, Square Inputs, Slew Rate, Joystick Deadband
        );
        public static final double gearRatio = 10.71;
        public static final double wheelDiameter = Units.inchesToMeters(6);
        public static final DifferentialDriveKinematics diffKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24.5));   //? Double check when drivebase built
        public static final Slot0Configs slot0ConfigMotionMagic = new Slot0Configs().withKP(0).withKD(0); //! Have to tune for these values
        // RoboRIO PID auto turning
        public static final SlotConfigs turnPIDConfig = new SlotConfigs().withKP(0).withKD(0); //! Have to tune for these values
        public static final double maxTurnSpeed = 0.25;
    }

    public static class Pivot {
        public static final int ID = 4;
        public static final int FID = 5;
        public static final double gearRatio = 100;
        public static final Slot0Configs slot0ConfigMotionMagic = new Slot0Configs().withKP(0).withKD(0); //! Have to tune for these values
        public static final double boreEncoderOffset = 0; //! Have to find this value
        public static final int stowAngle = 0; //! find angle for stow
        public static final double speakerAngle = 0; //! find angle for base speaker
        public static final double ampAngle = 0; //! find angle for amp
        public static final double pickupAngle = 0; //! find angle for pickup
    }

    public static class Intake {
        public static final int INID = 6;
        public static final int SID = 7;
        public static final Slot0Configs slot0Config = new Slot0Configs().withKP(0).withKD(0); //! Have to tune for these values
        public static final double speakerShootSpeed = 6380 / 60; // Max RPS of the Falcon 500
        public static final double ampShootSpeed = 0; //! Find this
        public static final double flywheelDiamter = Units.inchesToMeters(4); //! Find this
    }

    public static class Vision {
        public static final String cameraName = "LifeCam";
        public static final Transform3d robotToCamera = new Transform3d(); //! Have to configure
        public static final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }

    public static class Field {
        public static final double lowSpeakerHeight = Units.inchesToMeters(78);
        public static final Pose2d speakerPos = Vision.aprilTagFieldLayout.getTagPose(7).get().toPose2d();
    }

}
