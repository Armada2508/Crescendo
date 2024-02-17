package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.Constants.Shooter;
import frc.robot.lib.logging.Loggable;
import frc.robot.lib.logging.NTLogger;
import frc.robot.lib.music.TalonMusic;
import frc.robot.lib.util.Util;

public class IntakeShooterSubsystem extends SubsystemBase implements Loggable {

    private final TalonFX talonIntake = new TalonFX(Intake.intakeID);
    private final TalonFX talonShooter = new TalonFX(Shooter.shooterID); 
    private final TalonFX talonFollowShooter = new TalonFX(Shooter.shooterFollowID); 
    private final TimeOfFlight timeOfFlight = new TimeOfFlight(Intake.timeOfFlightID);

    public IntakeShooterSubsystem() {
        configTalons();
        NTLogger.register(this);
        TalonMusic.addTalonFX(this, talonIntake, talonShooter);
    }

    private void configTalons() {
        Util.factoryResetTalons(talonIntake, talonShooter, talonFollowShooter);
        Util.coastMode(talonIntake, talonShooter, talonFollowShooter);
        talonShooter.setInverted(true);
        talonFollowShooter.setControl(new StrictFollower(talonShooter.getDeviceID()));
    }

    public void setIntakeSpeed(double speed) {
        talonIntake.setControl(new DutyCycleOut(speed));
    }

    public void setShooterSpeed(double speed) {
        talonShooter.setControl(new DutyCycleOut(speed));
    }

    public void setShooterVoltage(Measure<Voltage> volts) {
        talonShooter.setControl(new VoltageOut(volts.in(Volts)));
    }

    public void stop() {
        talonIntake.setControl(new NeutralOut());
        talonShooter.setControl(new NeutralOut());
    }

    public boolean isSensorTripped() {
        return Util.inRange(timeOfFlight.getRange(), Intake.noteDetectionRange.in(Millimeters));
    }

    public Command intakeCommand() {
        return runOnce(() -> {
            setIntakeSpeed(Intake.intakeSpeed);
        })
        .andThen(Commands.waitUntil(this::isSensorTripped))
        .andThen(() -> setIntakeSpeed(-Intake.intakeSpeed))
        .andThen(Commands.waitUntil(() -> !isSensorTripped()))
        .finallyDo(this::stop)
        .withName("Intake Command");
    }

    public Command spinUpFlywheelCommand(Measure<Voltage> voltage) {
        return runOnce(() -> {
            setShooterVoltage(voltage);
        })
        .andThen(
            Commands.waitSeconds(Shooter.flywheelChargeTime.in(Seconds))
        )
        .withName("Spin Up Flywheel Command");
    }

    public Command releaseNoteCommand() {
        return runOnce(() -> setIntakeSpeed(Shooter.indexSpeed))
        .andThen(Commands.waitSeconds(Shooter.timeToShoot.in(Seconds))) 
        .finallyDo(this::stop)
        .withName("Release Note Command");
    }
    
    public Command shootCommand(Measure<Voltage> voltage) {
        return spinUpFlywheelCommand(voltage)
        .andThen(releaseNoteCommand())
        .withName("Shoot Command");
    }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        map.put("TOF Distance MM", timeOfFlight.getRange());
        map.put("Is TOF Tripped", isSensorTripped());
        map.put("Shooter RPM", talonShooter.getVelocity().getValueAsDouble() * 60);
        NTLogger.putTalonLog(talonIntake, map);
        NTLogger.putTalonLog(talonShooter, map);
        NTLogger.putTalonLog(talonFollowShooter, map);
        NTLogger.putSubsystemLog(this, map);
        return map;
    }

}
