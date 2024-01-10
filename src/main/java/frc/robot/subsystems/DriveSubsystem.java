package frc.robot.subsystems;

import static frc.robot.Constants.Drive.LFID;
import static frc.robot.Constants.Drive.LID;
import static frc.robot.Constants.Drive.RFID;
import static frc.robot.Constants.Drive.RID;
import static frc.robot.Constants.Drive.driveConfig;
import static frc.robot.Constants.Drive.gearRatio;
import static frc.robot.Constants.Drive.pigeonID;
import static frc.robot.Constants.Drive.wheelDiameter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.Encoder;
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

    @Override
    public void periodic() {}

    public void configTalons() {
        pigeon.configFactoryDefault();
        Util.factoryResetTalons(talonL, talonR, talonLF, talonRF);
        Util.brakeMode(talonL, talonR, talonLF, talonRF);
        talonLF.setControl(new StrictFollower(talonL.getDeviceID()));
        talonRF.setControl(new StrictFollower(talonR.getDeviceID()));
        talonLF.setInverted(true);
        talonRF.setInverted(true);
    }

    private void configMotionMagic(double velocity, double acceleration) {
        MotionMagicConfigs config = new MotionMagicConfigs();
        config.MotionMagicCruiseVelocity = Encoder.fromVelocity(velocity, gearRatio, wheelDiameter);
        config.MotionMagicAcceleration = Encoder.fromVelocity(acceleration, gearRatio, wheelDiameter);
        talonL.getConfigurator().apply(config);
        talonR.getConfigurator().apply(config);
    }

    public void setSpeed(double leftSpeed, double rightSpeed) {
        talonL.set(leftSpeed);
        talonR.set(rightSpeed);
    }

    /**
     * Drives the robot a certain distance relative to itself using motion magic
     * @param meters - distance to travel
     */
    private void driveDistance(double meters) {
        double distance = Encoder.fromDistance(meters, gearRatio, wheelDiameter);
        MotionMagicVoltage request = new MotionMagicVoltage(distance);
        talonL.setControl(request);
        talonR.setControl(request);
    }

    public Command driveDistanceCommand(double distance, double velocity, double acceleration) {
        // todo implement deadband + ending condition
        final double deadbandMeters = Units.inchesToMeters(0.5); 
        return runOnce(() -> {
            configMotionMagic(velocity, acceleration);
            driveDistance(distance);
        }).finallyDo(this::stop);
    }

    public void stop() {
        talonL.setControl(new NeutralOut());
        talonR.setControl(new NeutralOut());
    }

    public Command getDriveCommand(DoubleSupplier joystickSpeed, DoubleSupplier joystickTurn, DoubleSupplier joystickTrim, BooleanSupplier joystickSlow) {
        return new ButterySmoothDriveCommand(joystickSpeed, joystickTurn, joystickTrim, joystickSlow, driveConfig, this::setSpeed, this::stop, this);
    }

}
