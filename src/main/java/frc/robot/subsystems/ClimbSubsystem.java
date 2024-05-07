package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.lib.logging.NTLogger.log;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climb;
import frc.robot.lib.util.Util;

public class ClimbSubsystem extends SubsystemBase {

    private final TalonFX talon = new TalonFX(Climb.climbID);
    private final TalonFX talonFollow = new TalonFX(Climb.climbFollowID);

    public ClimbSubsystem() {
        configTalons();
    }

    @Override
    public void periodic() {
        logClimb();
    }

    private void configTalons() {
        Util.factoryResetTalons(talon, talonFollow);
        Util.brakeMode(talon, talonFollow);
        talon.setInverted(true);
        talon.setPosition(Climb.minPosition.in(Rotations));
        talon.getConfigurator().apply(Climb.softLimitSwitchConfig);
        talonFollow.setControl(new StrictFollower(talon.getDeviceID()));
    }

    public Command setVoltage(Measure<Voltage> volts) {
        return runOnce(() -> talon.setControl(new VoltageOut(volts.in(Volts))));
    }

    public Command resetClimberCommand() {
        return runOnce(() -> {
            talon.getConfigurator().apply(Climb.softLimitSwitchConfig.withReverseSoftLimitEnable(false));
            talonFollow.setControl(new VoltageOut(Climb.zeroVoltage.in(Volts)));
            talon.setControl(new VoltageOut(Climb.zeroVoltage.in(Volts)));
        })
        .andThen(
            Commands.waitSeconds(0.5), // Filter out inital current spike
            Commands.waitUntil(() -> Math.abs(talon.getTorqueCurrent().getValueAsDouble()) > Climb.zeroTripTorqueCurrent.in(Amps)).finallyDo(this::stop)
                .alongWith(Commands.waitUntil(() -> Math.abs(talonFollow.getTorqueCurrent().getValueAsDouble()) > Climb.zeroTripTorqueCurrent.in(Amps))
                .finallyDo(() -> talonFollow.setControl(new NeutralOut())))
        )
        .finallyDo(() -> {
            talonFollow.setControl(new StrictFollower(talon.getDeviceID()));
            stop();
            talon.setPosition(Climb.minPosition.in(Rotations));
            talon.getConfigurator().apply(Climb.softLimitSwitchConfig.withReverseSoftLimitEnable(true));
        })
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Reset Climber");
    }

    public void stop() {
        talon.setControl(new NeutralOut());
    }

    public Measure<Angle> getPosition() {
        return Rotations.of(talon.getPosition().getValueAsDouble());
    }

    private void logClimb() {
        log(this, "Climb TalonFX", talon);
        log(this, "Climb Follow TalonFX", talonFollow);
        log(this, "Subsystem", this);
    }
    
}
