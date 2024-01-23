package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeShooter;
import frc.robot.lib.util.Util;

public class IntakeShooterSubsystem extends SubsystemBase {

    private final TalonFX talonIntake = new TalonFX(IntakeShooter.intakeID);
    private final TalonFX talonShooter = new TalonFX(IntakeShooter.shooterID); 
    private final TimeOfFlight timeOfFlight = new TimeOfFlight(IntakeShooter.timeOfFlightID);

    public IntakeShooterSubsystem() {
        configTalons();
    }

    private void configTalons() {
        Util.factoryResetTalons(talonIntake, talonShooter);
        Util.coastMode(talonIntake, talonShooter);
        talonShooter.getConfigurator().apply(IntakeShooter.shooterVelocityConfig);
    }

    public void setIntakeSpeed(double speed) {
        talonIntake.setControl(new DutyCycleOut(speed));
    }

    public void setShooterSpeed(double speed) {
        talonShooter.setControl(new DutyCycleOut(speed));
    }

    public void stop() {
        talonIntake.setControl(new NeutralOut());
        talonShooter.setControl(new NeutralOut());
    }

    public boolean isSensorTripped() {
        return Util.inRange(timeOfFlight.getRange(), IntakeShooter.noteDetectionRange.in(Millimeters));
    }

    public Command intakeCommand() {
        return runOnce(() -> {
            setIntakeSpeed(IntakeShooter.intakeSpeed);
        })
        .andThen(Commands.waitUntil(this::isSensorTripped))
        .andThen(Commands.waitSeconds(IntakeShooter.waitTimeAfterTrip.in(Seconds)))
        .finallyDo(this::stop);
    }
    
    /**
     * @param velocity rotations per second
     */
    public Command shootCommand(double velocity) {
        return runOnce(() -> {
            final VelocityVoltage request = new VelocityVoltage(velocity);
            talonShooter.setControl(request);
        })
        .andThen(Commands.waitUntil(() -> Util.epsilonEquals(talonShooter.getVelocity().getValueAsDouble(), velocity, IntakeShooter.velocityDeadband.in(RotationsPerSecond))))
        .andThen(runOnce(() -> setIntakeSpeed(IntakeShooter.indexSpeed)))
        .andThen(Commands.waitSeconds(IntakeShooter.timeToShoot.in(Seconds))) 
        .finallyDo(this::stop);
    }

}
