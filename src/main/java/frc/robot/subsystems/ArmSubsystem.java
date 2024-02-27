package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Shooter;
import frc.robot.lib.logging.Loggable;
import frc.robot.lib.logging.NTLogger;
import frc.robot.lib.music.TalonMusic;
import frc.robot.lib.util.Util;

public class ArmSubsystem extends SubsystemBase implements Loggable {
    
    private final TalonFX talon = new TalonFX(Arm.ID);
    private final TalonFX talonFollow = new TalonFX(Arm.followerID);
    private final DutyCycleEncoder throughBoreEncoder = new DutyCycleEncoder(Arm.throughBoreEncoderID);
    /**Distance (in.), Angle (deg.) */
    private final InterpolatingDoubleTreeMap interpolatingAngleMap = new InterpolatingDoubleTreeMap(); 
    private boolean initalizedArm = false;

    public ArmSubsystem() {
        configTalons();
        NTLogger.register(this);
        TalonMusic.addTalonFX(this, talon);
        // Green
        interpolatingAngleMap.put(Shooter.maxShootDistance.in(Inches), 52.0);
        interpolatingAngleMap.put(93.0, 51.0);
        interpolatingAngleMap.put(81.9, 50.0);
        interpolatingAngleMap.put(52.6, 40.0);
        // Orange
        // interpolatingAngleMap.put(77.4, 47.0);
        // interpolatingAngleMap.put(72.3, 45.0);
        // interpolatingAngleMap.put(67.5, 43.0);
        // interpolatingAngleMap.put(62.8, 41.0);
        // interpolatingAngleMap.put(57.0, 40.0);
        // interpolatingAngleMap.put(52.4, 39.0);
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
            Measure<Angle> pos = getBoreAngle();
            if (pos.gte(Arm.encoderAccountForSlack)) {
                pos = pos.minus(Arm.armSlack);
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

    private double getFeedforward() {
        // Not currently used.
        return Arm.gravityFeedforward * Math.cos(getAngle().in(Radians));
    }

    private void setAngle(Measure<Angle> angle) {
        if (!initalizedArm) return;
        if (angle.lt(Arm.minAngle)) angle = Arm.minAngle;
        if (angle.gt(Arm.maxAngle)) angle = Arm.maxAngle;
        MotionMagicVoltage request = new MotionMagicVoltage(angle.in(Rotations)).withFeedForward(getFeedforward());
        talon.setControl(request);
    }

    public Command setSpeed(double speed) {
        return runOnce(() -> talon.setControl(new DutyCycleOut(speed)));
    }

    public void stop() {
        talon.setControl(new NeutralOut());
    }

    public Measure<Angle> getBoreAngle() {
        return Degrees.of((Rotations.of(throughBoreEncoder.getAbsolutePosition()).in(Degrees) + Arm.encoderOffset.in(Degrees)) % Constants.degreesPerRotation);
    }

    public Measure<Angle> getAngle() {
        return Rotations.of(talon.getPosition().getValueAsDouble());
    }

    public Measure<Angle> getPredictedAngle(Measure<Distance> distance) {
        return Degrees.of(interpolatingAngleMap.get(distance.in(Inches)));
    }

    private Measure<Angle> targetAngle; 
    /**
     * @param angle
     * @param velocity rotations per second
     * @param acceleration rotations per second^2
     * @param jerk rotations per second^3
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
     * @param velocity rotations per second
     * @param acceleration rotations per second^2
     * @param jerk rotations per second^3
     */
    public Command setAngleCommand(Measure<Angle> angle, double velocity, double acceleration, double jerk) {
        return setAngleCommand(() -> angle, velocity, acceleration, jerk);
    }

    public Command setAngleCommand(Measure<Angle> angle) {
        return setAngleCommand(angle, Arm.defaultVelocity, Arm.defaultAcceleration, Arm.defaultJerk);
    }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        map.put("Arm Angle", getAngle().in(Degrees));
        map.put("Arm Initalized", initalizedArm);
        map.put("Bore Encoder Connected", throughBoreEncoder.isConnected());
        map.put("Bore Encoder Angle Deg", getBoreAngle().in(Degrees));
        NTLogger.putTalonLog(talon, "Arm TalonFX", map);
        NTLogger.putTalonLog(talonFollow, "Arm Follow TalonFX", map);
        NTLogger.putSubsystemLog(this, map);
        return map;
    }

}
