package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Pivot;
import frc.robot.lib.Encoder;
import frc.robot.lib.util.Util;


public class PivotSubsystem extends SubsystemBase {
    
    private final TalonFX talon = new TalonFX(Pivot.ID);
    private final TalonFX talonF = new TalonFX(Pivot.FID);
    private final DutyCycleEncoder throughBoreEncoder = new DutyCycleEncoder(0);

    public PivotSubsystem() {
        configTalons();
    }

    private void configTalons() {
        Util.factoryResetTalons(talon, talonF);
        Util.brakeMode(talon, talonF);
        talonF.setControl(new StrictFollower(talon.getDeviceID()));
        talon.getConfigurator().apply(Pivot.slot0ConfigMotionMagic);
        talon.setPosition(throughBoreEncoder.getDistance() + Pivot.boreEncoderOffset);
    }

    /**
     * 
     * @param velocity rps
     * @param acceleration rps/s
     */
    public void configMotionMagic(double velocity, double acceleration) {
        MotionMagicConfigs config = new MotionMagicConfigs();
        config.MotionMagicAcceleration = Encoder.fromRotationalAngle(acceleration, Pivot.gearRatio);
        config.MotionMagicCruiseVelocity = Encoder.fromRotationalAngle(velocity, Pivot.gearRatio);;
        talon.getConfigurator().apply(config);
    }

    public void setSpeed(double speed) {
        talon.set(speed);
    }

    public void stop() {
        talon.setControl(new NeutralOut());
    }
    
    public void setAngle(DoubleSupplier degree) {
        double angleRots = Encoder.fromRotationalAngle(degree.getAsDouble(), Pivot.gearRatio);
        MotionMagicVoltage request = new MotionMagicVoltage(angleRots);
        talon.setControl(request);
    }
    
    /**
     * 
     * @param degree degrees
     * @param velocity rps
     * @param acceleration rps/s
     * @return
     */
    public Command setAngleCommand(DoubleSupplier degree, double velocity, double acceleration) {
        final double deadbandRotations = Encoder.fromRotationalAngle(0.5, Pivot.gearRatio); //! test degree
        return runOnce(() -> {
            configMotionMagic(velocity, acceleration);
            setAngle(degree);
        })
        .andThen(Commands.waitUntil(() -> Util.inRange(talon.getPosition().getValueAsDouble() - talon.getClosedLoopReference().getValueAsDouble(), deadbandRotations)))
        .finallyDo(this::stop);
    }

    public Command setAngleCommand(double degree, double velocity, double acceleration) {
        return setAngleCommand(() -> degree, velocity, acceleration);
    }

}
