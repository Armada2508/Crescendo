package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.lib.logging.NTLogger.log;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.lib.logging.NTLogger;
import frc.robot.lib.music.TalonMusic;
import frc.robot.lib.util.Util;

public class ArmSubsystem extends SubsystemBase {
    
    private final TalonFX talon = new TalonFX(Arm.ID);
    private final TalonFX talonFollow = new TalonFX(Arm.followerID);
    private final DutyCycleEncoder throughBoreEncoder = new DutyCycleEncoder(Arm.throughBoreEncoderID);
    private boolean initalizedArm = false;

    public ArmSubsystem() {
        configTalons();
        TalonMusic.addTalonFX(this, talon);
    }

    @Override
    public void periodic() {
        System.out.println(talon.getClosedLoopReference());
        logArm();
    }

    private void configTalons() {
        Util.factoryResetTalons(talon, talonFollow);
        Util.brakeMode(talon, talonFollow);
        talonFollow.setControl(new StrictFollower(talon.getDeviceID()));
        talon.setInverted(true);
        talon.getConfigurator().apply(Arm.motionMagicConfig);
        talon.getConfigurator().apply(Arm.feedbackConfig); // Applies gearbox ratio
        talon.getConfigurator().apply(Arm.softLimitSwitchConfig); // Soft Limits
        if (throughBoreEncoder.isConnected()) { 
            Measure<Angle> pos = getBoreEncoderAngle();
            if (pos.gte(Arm.boreEncoderHardstop)) {
                pos = Arm.maxAngle;
            }
            talon.setPosition(pos.in(Rotations));
            initalizedArm = true;
        }
    }

    /**
     * @param velocity degrees per second
     * @param acceleration degrees per second^2
     * @param jerk degrees per second^3
     */
    private void configMotionMagic(double velocity, double acceleration, double jerk) {
        MotionMagicConfigs config = new MotionMagicConfigs();
        config.MotionMagicCruiseVelocity = velocity / Constants.degreesPerRotation;
        config.MotionMagicAcceleration = acceleration  / Constants.degreesPerRotation;
        config.MotionMagicJerk = jerk / Constants.degreesPerRotation;
        talon.getConfigurator().apply(config);
    }

    private void setAngle(Measure<Angle> angle) {
        if (!initalizedArm) return;
        if (angle.lt(Arm.minAngle)) angle = Arm.minAngle;
        if (angle.gt(Arm.maxAngle)) angle = Arm.maxAngle;
        MotionMagicVoltage request = new MotionMagicVoltage(angle.in(Rotations));
        StatusCode code = talon.setControl(request);
        NTLogger.log(this, "Last Status Code", code);
    }

    public Command setVoltage(Measure<Voltage> volts) {
        return runOnce(() -> talon.setControl(new VoltageOut(volts.in(Volts))));
    }

    public void stop() {
        talon.setControl(new NeutralOut());
    }

    public Measure<Angle> getBoreEncoderAngle() {
        return Degrees.of((Rotations.of(throughBoreEncoder.getAbsolutePosition()).in(Degrees) + Arm.encoderOffset.in(Degrees)) % Constants.degreesPerRotation);
    }

    public Measure<Angle> getAngle() {
        return Rotations.of(talon.getPosition().getValueAsDouble());
    }

    private Measure<Angle> targetAngle = getAngle();
    /**
     * @param angle
     * @param velocity degrees per second
     * @param acceleration degrees per second^2
     * @param jerk degrees per second^3
     */
    public Command setAngleCommand(Supplier<Measure<Angle>> angle, double velocity, double acceleration, double jerk) {
        final double deadband = Arm.angleDeadband.in(Degrees); 
        return runOnce(() -> {
            targetAngle = angle.get();
            configMotionMagic(velocity, acceleration, jerk);
            setAngle(targetAngle);
        })
        .andThen(Commands.waitUntil(() -> Util.inRange(getAngle().minus(targetAngle).in(Degrees), deadband))) 
        .withName("Set Angle");
    }

    public Command setAngleCommand(Supplier<Measure<Angle>> angle) {
        return setAngleCommand(angle, Arm.defaultVelocity, Arm.defaultAcceleration, Arm.defaultJerk);
    }

    /**
     * @param angle
     * @param velocity degrees per second
     * @param acceleration degrees per second^2
     * @param jerk degrees per second^3
     */
    public Command setAngleCommand(Measure<Angle> angle, double velocity, double acceleration, double jerk) {
        return setAngleCommand(() -> angle, velocity, acceleration, jerk);
    }

    public Command setAngleCommand(Measure<Angle> angle) {
        return setAngleCommand(angle, Arm.defaultVelocity, Arm.defaultAcceleration, Arm.defaultJerk);
    }

    private void logArm() {
        log(this, "Arm Angle", getAngle().in(Degrees));
        log(this, "Arm Initalized", initalizedArm);
        log(this, "Bore Encoder Connected", throughBoreEncoder.isConnected());
        log(this, "Bore Encoder Angle Deg", getBoreEncoderAngle().in(Degrees));
        log(this, "Target Angle", targetAngle.in(Degrees));
        log(this, "Arm TalonFX", talon);
        log(this, "Arm Follow TalonFX", talonFollow);
        log(this, "Subsystem", this);
    }

}
