package frc.robot.subsystems;

import static frc.robot.Constants.Drive.gearRatio;
import static frc.robot.Constants.Pivot.FID;
import static frc.robot.Constants.Pivot.ID;
import static frc.robot.Constants.Pivot.slot0ConfigMotionMagic;

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
    
    final TalonFX talon = new TalonFX(ID);
    final TalonFX talonF = new TalonFX(FID);
    private final DutyCycleEncoder throughBoreEncoder = new DutyCycleEncoder(0);

    public PivotSubsystem() {
        configTalons();
    }

    public void configTalons() {
        Util.factoryResetTalons(talon, talonF);
        Util.brakeMode(talon, talonF);
        talonF.setControl(new StrictFollower(talon.getDeviceID()));
        talon.getConfigurator().apply(slot0ConfigMotionMagic);
        talon.setPosition(throughBoreEncoder.getDistance() + Pivot.boreEncoderOffset);
    }

    public void configMotionMagic(double degree) {
        MotionMagicConfigs config = new MotionMagicConfigs();
        config.MotionMagicAcceleration = Encoder.fromRotationalAngle(degree, gearRatio);
        config.MotionMagicCruiseVelocity = Encoder.fromRotationalAngle(degree, gearRatio);;
        talon.getConfigurator().apply(config);
    }

    public void setSpeed(double speed) {
        talon.set(speed);
    }

    public void stop() {
        talon.setControl(new NeutralOut());
    }
    
    public void setAngle(double degree) {
        double angleRots = Encoder.fromRotationalAngle(degree, gearRatio);
        MotionMagicVoltage request = new MotionMagicVoltage(angleRots);
        talon.setControl(request);
    }

    public Command setAngleCommand(double degree) {
        final double deadbandRotations = Encoder.fromRotationalAngle(degree, gearRatio);
        return runOnce(() -> {
            configMotionMagic(degree);
            setAngle(degree);
        })
        .andThen(Commands.waitUntil(() -> Util.inRange(talon.getPosition().getValueAsDouble() - talon.getClosedLoopReference().getValueAsDouble(), deadbandRotations)))
        .finallyDo(this::stop);
    }
}
