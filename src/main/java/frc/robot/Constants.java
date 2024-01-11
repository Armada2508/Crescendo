package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.util.Units;
import frc.robot.lib.drive.ButterySmoothDriveCommand.DriveConfig;

public class Constants {
    
    public static class Drive {
        public static final int LID = 0;
        public static final int LFID = 1;
        public static final int RID = 2;
        public static final int RFID = 3;
        public static final int pigeonID = 9;
        public static final DriveConfig driveConfig = new DriveConfig(
            1, 1, 1, 0.5, true, 1.5, 0.07 // Speed Multi, Turn Multi, Trim Multi, Slow Speed, Square Inputs, Slew Rate, Joystick Deadband
        );
        public static final double gearRatio = 10.71;
        public static final double wheelDiameter = Units.inchesToMeters(6);
        public static final Slot0Configs slot0Config = new Slot0Configs().withKP(0).withKD(0); //! Have to tune for these values
    }

    public static class Pivot {
        public static final int ID = 0;
        public static final int FID = 1;
    }

}
