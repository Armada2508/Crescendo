package frc.robot.subsystems;

import static frc.robot.Constants.Drive.LFID;
import static frc.robot.Constants.Drive.LID;
import static frc.robot.Constants.Drive.RFID;
import static frc.robot.Constants.Drive.RID;
import static frc.robot.Constants.Drive.gearRatio;
import static frc.robot.Constants.Drive.joystickDriveConfig;
import static frc.robot.Constants.Drive.maxTurnSpeed;
import static frc.robot.Constants.Drive.pigeonID;
import static frc.robot.Constants.Drive.slot0ConfigMotionMagic;
import static frc.robot.Constants.Drive.turnPIDConfig;
import static frc.robot.Constants.Drive.wheelDiameter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    private final PIDController turnPIDController = new PIDController(turnPIDConfig.kP, turnPIDConfig.kI, turnPIDConfig.kD);
    
    public DriveSubsystem() {
        turnPIDController.setTolerance(0.5); // degrees
        configTalons();
    }

    @Override
    public void periodic() {}

    private void configTalons() {
        pigeon.configFactoryDefault();
        pigeon.setYaw(0);
        Util.factoryResetTalons(talonL, talonR, talonLF, talonRF);
        Util.brakeMode(talonL, talonR, talonLF, talonRF);
        talonLF.setControl(new StrictFollower(talonL.getDeviceID()));
        talonRF.setControl(new StrictFollower(talonR.getDeviceID()));
        talonLF.setInverted(true);
        talonRF.setInverted(true);
        talonL.getConfigurator().apply(slot0ConfigMotionMagic);
        talonR.getConfigurator().apply(slot0ConfigMotionMagic);
    }

    /**
     * 
     * @param velocity meters/second
     * @param acceleration meters/second^2
     */
    private void configMotionMagic(double velocity, double acceleration) {
        MotionMagicConfigs config = new MotionMagicConfigs();
        config.MotionMagicCruiseVelocity = Encoder.fromVelocity(velocity, gearRatio, wheelDiameter);
        config.MotionMagicAcceleration = Encoder.fromVelocity(acceleration, gearRatio, wheelDiameter);
        talonL.getConfigurator().apply(config);
        talonR.getConfigurator().apply(config);
    }

    /**
     * Drives the robot a certain distance relative to itself using motion magic
     * @param meters - distance to travel
     */
    private void driveDistance(double meters) {
        double distanceRots = Encoder.fromDistance(meters, gearRatio, wheelDiameter);
        MotionMagicVoltage request = new MotionMagicVoltage(distanceRots);
        talonL.setControl(request);
        talonR.setControl(request);
    }

    public void setSpeed(double leftSpeed, double rightSpeed) {
        talonL.set(leftSpeed);
        talonR.set(rightSpeed);
    }

    /**
     * 
     * @param distance meters relative to robot
     * @param velocity meters/second
     * @param acceleration meters/second^2
     * @return Command for the robot to drive distance using motion magic
     */
    public Command driveDistanceCommand(double distance, double velocity, double acceleration) {
        final double deadbandRotations = Encoder.fromDistance(Units.inchesToMeters(0.5), gearRatio, wheelDiameter); 
        return runOnce(() -> {
            configMotionMagic(velocity, acceleration);
            driveDistance(distance);
        })
        .andThen(Commands.waitUntil(() -> Util.inRange(talonL.getPosition().getValueAsDouble() - talonL.getClosedLoopReference().getValueAsDouble(), deadbandRotations)))
        .finallyDo(this::stop);
    }

    /**
     * 
     * @param angle - degrees relative to front of robot 
     * @return Command for the robot to turn amount of degrees relative to front of robot
     */
    public Command turnCommand(double angle) {
        turnPIDController.reset();
        turnPIDController.setSetpoint(pigeon.getYaw() + angle);
        return runEnd(() -> {
            double speed = turnPIDController.calculate(pigeon.getYaw());
            speed = MathUtil.clamp(speed, -maxTurnSpeed, maxTurnSpeed);
            setSpeed(speed, -speed);
        }, this::stop).until(turnPIDController::atSetpoint);
    }

    public void stop() {
        talonL.setControl(new NeutralOut());
        talonR.setControl(new NeutralOut());
    }

    public Command joystickDriveCommand(DoubleSupplier joystickSpeed, DoubleSupplier joystickTurn, DoubleSupplier joystickTrim, BooleanSupplier joystickSlow) {
        return new ButterySmoothDriveCommand(joystickSpeed, joystickTurn, joystickTrim, joystickSlow, joystickDriveConfig, this::setSpeed, this::stop, this);
    }

}
