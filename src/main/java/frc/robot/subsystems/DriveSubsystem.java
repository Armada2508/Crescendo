package frc.robot.subsystems;

import static frc.robot.Constants.Drive.LFID;
import static frc.robot.Constants.Drive.LID;
import static frc.robot.Constants.Drive.RFID;
import static frc.robot.Constants.Drive.RID;
import static frc.robot.Constants.Drive.driveConfig;
import static frc.robot.Constants.Drive.pigeonID;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.drive.ButterySmoothDriveCommand;
import frc.robot.lib.util.Util;

public class DriveSubsystem extends SubsystemBase {

    private final TalonFX talonL = new TalonFX(LID);
    private final TalonFX talonR = new TalonFX(RID);
    private final TalonFX talonLF = new TalonFX(LFID);
    private final TalonFX talonRF = new TalonFX(RFID);
    private final PigeonIMU pigeon = new PigeonIMU(pigeonID);
    
    public DriveSubsystem() {
        configTalons();
    }

    public void configTalons() {
        pigeon.configFactoryDefault();
        Util.factoryResetTalons(talonL, talonR, talonLF, talonRF);
        Util.brakeMode(talonL, talonR, talonLF, talonRF);
        talonLF.setControl(new StrictFollower(talonL.getDeviceID()));
        talonRF.setControl(new StrictFollower(talonR.getDeviceID()));
        talonLF.setInverted(true);
        talonRF.setInverted(true);
    }

    public void setPower(double leftPower, double rightPower) {
        talonL.set(leftPower);
        talonR.set(rightPower);
    }

    public void stop() {
        talonL.setControl(new NeutralOut());
        talonR.setControl(new NeutralOut());
    }

    @Override
    public void periodic() {}

    public Command getDriveCommand(DoubleSupplier joystickSpeed, DoubleSupplier joystickTurn, DoubleSupplier joystickTrim, BooleanSupplier joystickSlow) {
        return new ButterySmoothDriveCommand(joystickSpeed, joystickTurn, joystickTrim, joystickSlow, driveConfig, this::setPower, this::stop, this);
    }

}
