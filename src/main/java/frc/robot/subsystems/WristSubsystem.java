package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

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
import frc.robot.Constants.Wrist;
import frc.robot.lib.logging.Loggable;
import frc.robot.lib.logging.NTLogger;
import frc.robot.lib.util.Util;

public class WristSubsystem extends SubsystemBase implements Loggable {

    private final TalonFX talon = new TalonFX(Wrist.ID);
    private final TalonFX talonFollow = new TalonFX(Wrist.followID);
    private final DutyCycleEncoder boreEncoder = new DutyCycleEncoder(Wrist.boreEncoderID);
    private boolean initalizedWrist = false;
    
    public WristSubsystem() {
        configTalons();
        NTLogger.register(this);
    }

    private void configTalons() { 
        Util.factoryResetTalons(talon, talonFollow);
        Util.brakeMode(talon, talonFollow);
        talonFollow.setControl(new StrictFollower(talon.getDeviceID()));
        talon.getConfigurator().apply(Wrist.motionMagicConfig);
        talon.getConfigurator().apply(Wrist.feedbackConfig);
        talon.getConfigurator().apply(Wrist.softLimitSwitchConfig);
        if (boreEncoder.isConnected()) {
            Measure<Angle> pos = getBoreEncoderAngle();
            talon.setPosition(pos.in(Rotations));
            initalizedWrist = true;
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

    public Measure<Angle> getBoreEncoderAngle() {
        return Degrees.of((Rotations.of(boreEncoder.getAbsolutePosition()).in(Degrees) + Wrist.encoderOffset.in(Degrees)) % Constants.degreesPerRotation);
    }

    private void setAngle(Measure<Angle> angle) {
        if (angle.lt(Wrist.minAngle)) angle = Wrist.minAngle;
        if (angle.gt(Wrist.maxAngle)) angle = Wrist.maxAngle;
        MotionMagicVoltage request = new MotionMagicVoltage(angle.in(Rotations));
        talon.setControl(request);
    }

    public Command setVoltage(Measure<Voltage> volts) {
        return runOnce(() -> talon.setControl(new VoltageOut(volts.in(Volts))));
    }

    public void stop() {
        talon.setControl(new NeutralOut());
    }

    public Measure<Angle> getAngle() {
        return Rotations.of(talon.getPosition().getValueAsDouble());
    }

    public Command setAngleCommand(Measure<Angle> angle, double velocity, double acceleration, double jerk) {
        final double deadband = Wrist.angleDeadband.in(Degrees);
        return runOnce(() -> {
            configMotionMagic(velocity, acceleration, jerk);
            setAngle(angle);
        })
        .andThen(Commands.waitUntil(() -> Util.inRange(getAngle().minus(angle).in(Degrees), deadband)))
        .finallyDo(this::stop).withName("Set Angle");
    }

    public Command setAngleCommand(Measure<Angle> angle) {
        return setAngleCommand(angle, Wrist.defaultVelocity, Wrist.defaultAcceleration, Wrist.defaultJerk);
    }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        map.put("Wrist Angle", getAngle().in(Degrees));
        map.put("Wrist Initalized", initalizedWrist);
        map.put("Bore Encoder Connected", boreEncoder.isConnected());
        map.put("Bore Encoder Angle Deg", getBoreEncoderAngle().in(Degrees));
        NTLogger.putTalonLog(talon, "Wrist TalonFX", map);
        NTLogger.putTalonLog(talonFollow, "Wrist Follow TalonFX", map);
        NTLogger.putSubsystemLog(this, map);
        return map;
    }
}
