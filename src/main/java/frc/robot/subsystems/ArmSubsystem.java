package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;
import java.util.function.BooleanSupplier;
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
import frc.robot.lib.logging.Loggable;
import frc.robot.lib.logging.NTLogger;
import frc.robot.lib.music.TalonMusic;
import frc.robot.lib.util.Util;

public class ArmSubsystem extends SubsystemBase implements Loggable {
    
    private final TalonFX talon = new TalonFX(Arm.ID);
    private final TalonFX talonFollow = new TalonFX(Arm.followerID);
    private final DutyCycleEncoder throughBoreEncoder = new DutyCycleEncoder(Arm.throughBoreEncoderID);
    private final BooleanSupplier isPistonExtended;
    private boolean initalizedArm = false;
    private StatusCode lastControlStatusCode = StatusCode.StatusCodeNotInitialized;

    public ArmSubsystem(BooleanSupplier isPistonExtended) {
        this.isPistonExtended = isPistonExtended;
        configTalons();
        NTLogger.register(this);
        TalonMusic.addTalonFX(this, talon);
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
        if (angle.lt(Arm.retractAngle) && !isPistonExtended.getAsBoolean()) {
            System.out.println("OH NO WE'RE GONNA BREAK THE ARM");
            return;
        }
        if (!initalizedArm) return;
        if (angle.lt(Arm.minAngle)) angle = Arm.minAngle;
        if (angle.gt(Arm.maxAngle)) angle = Arm.maxAngle;
        MotionMagicVoltage request = new MotionMagicVoltage(angle.in(Rotations));
        lastControlStatusCode = talon.setControl(request);
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

    public Command stowCommand() {
        return setAngleCommand(Arm.stowAngle).withName("Stow");
    }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        map.put("Arm Angle", getAngle().in(Degrees));
        map.put("Arm Initalized", initalizedArm);
        map.put("Bore Encoder Connected", throughBoreEncoder.isConnected());
        map.put("Bore Encoder Angle Deg", getBoreEncoderAngle().in(Degrees));
        map.put("Target Angle", targetAngle.in(Degrees));
        map.put("Control Status Code", lastControlStatusCode);
        NTLogger.putTalonLog(talon, "Arm TalonFX", map);
        NTLogger.putTalonLog(talonFollow, "Arm Follow TalonFX", map);
        NTLogger.putSubsystemLog(this, map);
        return map;
    }

}
