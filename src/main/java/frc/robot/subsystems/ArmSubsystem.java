package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
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
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.lib.logging.Loggable;
import frc.robot.lib.logging.NTLogger;
import frc.robot.lib.util.Util;

public class ArmSubsystem extends SubsystemBase implements Loggable {
    
    private final TalonFX talon = new TalonFX(Arm.ID);
    private final TalonFX talonFollow = new TalonFX(Arm.followID);
    private final Relay holdingSolenoid = new Relay(Arm.relayChannel);
    private final InterpolatingDoubleTreeMap interpolatingAngleMap = new InterpolatingDoubleTreeMap(); //! Have to fill this map

    public ArmSubsystem() {
        configTalons();
        NTLogger.register(this);
        // interpolatingAngleMap.put(0.0, 0.0);
    }

    private void configTalons() {
        Util.factoryResetTalons(talon, talonFollow);
        Util.brakeMode(talon, talonFollow);
        talonFollow.setControl(new StrictFollower(talon.getDeviceID()));
        talon.getConfigurator().apply(Arm.motionMagicConfig);
        talon.getConfigurator().apply(Arm.feedbackConfig); // Applies gearbox ratio
        talon.setPosition(Arm.startAngle.in(Rotations));
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
        return Arm.gravityFeedforward * Math.cos(getAngle().in(Radians));
    }

    private void setAngle(Supplier<Measure<Angle>> angleSupplier) {
        Measure<Angle> angle = angleSupplier.get();
        if (angle.lt(Arm.minAngle)) angle = Arm.minAngle;
        if (angle.gt(Arm.maxAngle)) angle = Arm.maxAngle;
        MotionMagicVoltage request = new MotionMagicVoltage(angle.in(Rotations)).withFeedForward(getFeedforward());
        talon.setControl(request);
    }

    public void setSpeed(double speed) {
        talon.setControl(new DutyCycleOut(speed));
    }

    public void switchRelay() {
        if (holdingSolenoid.get() == Value.kOff) {
            holdingSolenoid.set(Value.kForward);
        } 
        else {
            holdingSolenoid.set(Value.kOff);
        }
    }

    public void stop() {
        talon.setControl(new NeutralOut());
    }

    public Measure<Angle> getAngle() {
        return Degrees.of(talon.getPosition().getValueAsDouble());
    }

    public Measure<Angle> getTargetAngle(double distance) {
        return Degrees.of(interpolatingAngleMap.get(distance));
    }
    
    /**
     * 
     * @param angle degrees
     * @param velocity rotations per second
     * @param acceleration rotations per second^2
     * @return
     */
    public Command setAngleCommand(Supplier<Measure<Angle>> angle, double velocity, double acceleration, double jerk) {
        final double deadbandRotations = Arm.angleDeadband.in(Rotations); 
        return runOnce(() -> {
            configMotionMagic(velocity, acceleration, jerk);
            setAngle(angle);
        })
        .andThen(Commands.waitUntil(() -> Util.inRange(talon.getPosition().getValueAsDouble() - talon.getClosedLoopReference().getValueAsDouble(), deadbandRotations)))
        .withName("Set Angle Command");
    }

    public Command setAngleCommand(Measure<Angle> angle, double velocity, double acceleration, double jerk) {
        return setAngleCommand(() -> angle, velocity, acceleration, jerk);
    }

    public Command retractRelayHoldCommand() { 
        return runOnce(() -> {
            if (holdingSolenoid.get() == Value.kForward) {
                switchRelay();
            }
        })
        .andThen(setAngleCommand(getAngle(), 0, 0, 0)); 
    }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        NTLogger.putTalonLog(talon, map);
        NTLogger.putTalonLog(talonFollow, map);
        NTLogger.putSubsystemLog(this, map);
        return map;
    }

}
