package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Arm;
import frc.robot.lib.Encoder;
import frc.robot.lib.util.Util;


public class ArmSubsystem extends SubsystemBase {
    
    private final TalonFX talon = new TalonFX(Arm.ID);
    private final TalonFX talonFollow = new TalonFX(Arm.followID);
    private final DutyCycleEncoder throughBoreEncoder = new DutyCycleEncoder(Arm.throughBoreEncoderID);

    public ArmSubsystem() {
        configTalons();
    }

    private void configTalons() {
        Util.factoryResetTalons(talon, talonFollow);
        Util.brakeMode(talon, talonFollow);
        talonFollow.setControl(new StrictFollower(talon.getDeviceID()));
        talon.getConfigurator().apply(Arm.motionMagicConfig);
        double encoderPos = throughBoreEncoder.getDistance() * Constants.degreesPerRotation / Arm.boreEncoderTicksPerRotation; 
        double actualPos = encoderPos + Arm.boreEncoderOffset.in(Degrees);
        talon.setPosition(Encoder.fromRotationalAngle(actualPos, Arm.gearRatio));
    }

    /**
     * @param velocity rotations per second
     * @param acceleration rotations per second^2
     */
    private void configMotionMagic(double velocity, double acceleration) {
        MotionMagicConfigs config = new MotionMagicConfigs();
        config.MotionMagicCruiseVelocity = Encoder.fromRotationalAngle(velocity, Arm.gearRatio);
        config.MotionMagicAcceleration = Encoder.fromRotationalAngle(acceleration, Arm.gearRatio);
        talon.getConfigurator().apply(config);
    }

    private double getFeedforward() {
        return Arm.gravityFeedforward * Math.cos(getAngle());
    }

    private void setAngle(Supplier<Measure<Angle>> angleSupplier) {
        Measure<Angle> angle = angleSupplier.get();
        if (angle.lt(Arm.minAngle)) angle = Arm.minAngle;
        if (angle.gt(Arm.maxAngle)) angle = Arm.maxAngle;
        double angleRots = Encoder.fromRotationalAngle(angle.in(Degrees), Arm.gearRatio);
        MotionMagicVoltage request = new MotionMagicVoltage(angleRots).withFeedForward(getFeedforward());
        talon.setControl(request);
    }

    public void setSpeed(double speed) {
        talon.setControl(new DutyCycleOut(speed));
    }

    public void stop() {
        talon.setControl(new NeutralOut());
    }

    public double getAngle() {
        return Encoder.toRotationalAngle(talon.getPosition().getValueAsDouble(), Arm.gearRatio);
    }
    
    /**
     * 
     * @param angle degrees
     * @param velocity rotations per second
     * @param acceleration rotations per second^2
     * @return
     */
    public Command setAngleCommand(Supplier<Measure<Angle>> angle, double velocity, double acceleration) {
        final double deadbandRotations = Encoder.fromRotationalAngle(Arm.angleDeadband.in(Degrees), Arm.gearRatio); 
        return runOnce(() -> {
            configMotionMagic(velocity, acceleration);
            setAngle(angle);
        })
        .andThen(Commands.waitUntil(() -> Util.inRange(talon.getPosition().getValueAsDouble() - talon.getClosedLoopReference().getValueAsDouble(), deadbandRotations)));
    }

    public Command setAngleCommand(Measure<Angle> angle, double velocity, double acceleration) {
        return setAngleCommand(() -> angle, velocity, acceleration);
    }

}
