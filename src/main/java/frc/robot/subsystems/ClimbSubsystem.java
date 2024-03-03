package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Current;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Climb;
import frc.robot.lib.logging.Loggable;
import frc.robot.lib.logging.NTLogger;
import frc.robot.lib.util.Util;

public class ClimbSubsystem extends SubsystemBase  implements Loggable{

    private final TalonFX talon = new TalonFX(Climb.climbID);
    private final TalonFX talonFollow = new TalonFX(Climb.climbFollowID);

    public ClimbSubsystem() {
        configTalons();
        NTLogger.register(this);
    }

    public void configTalons() {
        Util.factoryResetTalons(talon, talonFollow);
        Util.brakeMode(talon, talonFollow);
        talonFollow.setInverted(true);
    }

    public Command setVoltage(Measure<Voltage> volts) {
        return runOnce(() -> talon.setControl(new VoltageOut(volts.in(Volts))));
    }

    public void stop() {
        talon.setControl(new NeutralOut());
    }

    public Command extendClimberCommand() {
        return setVoltage(Climb.climbPower)
        .andThen(Commands.waitSeconds(2))
        .andThen(this::stop)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Extend Climber");
    }

    public Command retractClimberCommand() {
        return setVoltage(Climb.climbPower.negate())
        .andThen(Commands.waitSeconds(2))
        .andThen(this::stop)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Retract Climber");
    }

    public Command resetClimberCommand() {
        Measure<Voltage> volts = Volts.of(-0.5);
        Measure<Current> tripCurrent = Amps.of(0.2);
        return runOnce(() -> {
            talon.setControl(new VoltageOut(volts.in(Volts)));
            talonFollow.setControl(new VoltageOut(volts.in(Volts)));
        })
        .andThen(Commands.waitUntil(() -> talon.getSupplyCurrent().getValueAsDouble() > tripCurrent.in(Amps))).finallyDo(this::stop)
        .alongWith(
            Commands.waitUntil(() -> talonFollow.getSupplyCurrent().getValueAsDouble() > tripCurrent.in(Amps)).finallyDo(() -> talonFollow.setControl(new NeutralOut()))
        )
        .andThen(() -> talonFollow.setControl(new StrictFollower(talon.getDeviceID())))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Reset Climber");
    }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        NTLogger.putTalonLog(talon, "Climb TalonFX", map);
        NTLogger.putTalonLog(talon, "Climb Follow TalonFX", map);
        NTLogger.putSubsystemLog(this, map);
        return map;
    }
}