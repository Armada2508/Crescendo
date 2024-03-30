package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
    private final TalonFX talonFollowShooter = new TalonFX(Shooter.shooterFollowerID); 
    private final TimeOfFlight timeOfFlight = new TimeOfFlight(Intake.timeOfFlightID);

    public IntakeShooterSubsystem() {
        configTalons();
        NTLogger.register(this);
        TalonMusic.addTalonFX(this, talonIntake, talonShooter);
    }

    private void configTalons() {
        Util.factoryResetTalons(talonIntake, talonShooter, talonFollowShooter);
        Util.coastMode(talonIntake, talonShooter, talonFollowShooter);
        talonFollowShooter.setInverted(true);
        talonFollowShooter.setControl(new StrictFollower(talonShooter.getDeviceID()));
    }

    public Command setIntakeSpeed(double speed) {
        return runOnce(() -> talonIntake.setControl(new DutyCycleOut(speed)));
    }

    public Command setIntakeVoltage(Measure<Voltage> volts) {
        return runOnce(() -> talonIntake.setControl(new VoltageOut(volts.in(Volts))));
    }

    public Command setShooterVoltage(Measure<Voltage> voltage) {
        return runOnce(() -> {
            talonFollowShooter.setControl(new StrictFollower(talonShooter.getDeviceID()));
            talonShooter.setControl(new VoltageOut(voltage.in(Volts)));
        });
    }

    public void stop() {
        talonIntake.setControl(new NeutralOut());
        talonShooter.setControl(new NeutralOut());
    }

    public Command brakeShooter() {
        return runOnce(() -> {
            var request = new StaticBrake();
            talonShooter.setControl(request);
            talonFollowShooter.setControl(request);
        })
        .withName("Brake Shooter");
    }

    public boolean isSensorTripped() {
        return Util.inRange(timeOfFlight.getRange(), Intake.noteDetectionRange.in(Millimeters));
    }

    public Command intakeCommand() {
        return Commands.waitUntil(() -> getShooterRPM().lte(Shooter.minShooterVelocityBraking)) 
        .andThen(
            brakeShooter(),
            setIntakeSpeed(Intake.intakeSpeed),
            Commands.waitUntil(this::isSensorTripped),
            Commands.waitSeconds(Intake.waitAfterTrip.in(Seconds)),
            runOnce(this::stop),
            Commands.waitSeconds(Intake.noteSettleTime.in(Seconds)),
            setIntakeSpeed(Intake.backOffSpeed),
            Commands.waitSeconds(Intake.backOffNoteTime.in(Seconds))
        ).finallyDo(this::stop)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
        .withName("Intake");
    }

    public Command spinUpFlywheelCommand() {
        return setShooterVoltage(Shooter.speakerShootPower)
        .andThen(
            Commands.waitUntil(() -> getShooterRPM().gte(Shooter.speakerShootVelocity))
            .withTimeout(Shooter.flywheelChargeTime.in(Seconds))
        ) 
        .withName("Spin Up Flywheel");
    }

    public Command releaseNoteCommand() {
        return setIntakeSpeed(Shooter.indexSpeed)
        .andThen(Commands.waitSeconds(Shooter.speakerTimeToShoot.in(Seconds))) 
        .finallyDo(this::stop)
        .withName("Release Note");
    }
    
    public Command shootSpeakerCommand() {
        return spinUpFlywheelCommand()
        .andThen(releaseNoteCommand())
        .withName("Shoot Speaker");
    }

    public Command shootAmpCommand() {
        return setIntakeVoltage(Shooter.ampShootPower)
        .andThen(Commands.waitSeconds(Shooter.ampTimeToShoot.in(Seconds)))
        .finallyDo(this::stop)
        .withName("Shoot Amp");
    }

    public Measure<Velocity<Angle>> getShooterRPM() {
        return RotationsPerSecond.of(talonShooter.getVelocity().getValueAsDouble());
    }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        map.put("TOF Status", timeOfFlight.getStatus().toString());
        map.put("TOF Distance MM", timeOfFlight.getRange());
        map.put("Is TOF Tripped", isSensorTripped());
        map.put("Shooter RPM", getShooterRPM().in(Rotations.per(Minute)));
        map.put("Shooter Follower RPM", talonFollowShooter.getVelocity().getValueAsDouble() * 60);
        map.put("Intake RPM", talonIntake.getVelocity().getValueAsDouble() * 60);
        NTLogger.putTalonLog(talonIntake, "Intake TalonFX", map);
        NTLogger.putTalonLog(talonShooter, "Shooter TalonFX", map);
        NTLogger.putTalonLog(talonFollowShooter, "Shooter Follow TalonFX", map);
        NTLogger.putSubsystemLog(this, map);
        return map;
    }

}
