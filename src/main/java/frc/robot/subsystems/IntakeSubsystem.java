package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.lib.util.Util;

public class IntakeSubsystem extends SubsystemBase{

    private final TalonFX talonIntake = new TalonFX(Intake.INID); // intaking / index
    private final TalonFX talonShoot = new TalonFX(Intake.SID); // shooting
    private final TimeOfFlight timeOfFlight = new TimeOfFlight(Intake.TimeOfFlightID);

    public IntakeSubsystem() {
        configTalons();
    }

    private void configTalons() {
        Util.factoryResetTalons(talonIntake, talonShoot);
        Util.coastMode(talonIntake, talonShoot);
        talonShoot.getConfigurator().apply(Intake.slot0Config);
    }

    public void setSpeed(double speed) {
        talonIntake.set(speed);
    }

    public void stop() {
        talonIntake.setControl(new NeutralOut());
        talonShoot.setControl(new NeutralOut());
    }
    /**
     * time of flight range is in milimeters
     * @return
     */
    public boolean isSensorTripped() {
        return Util.inRange(timeOfFlight.getRange(), Intake.fromSensorRange);
    }

    public Command intakeCommand() {
        return runOnce(() -> {
            setSpeed(0.5); //? possibly tune speed
        })
        .andThen(Commands.waitUntil(() -> isSensorTripped()))
        .andThen(Commands.waitSeconds(1)) //tune wait seconds for note
        .finallyDo(this::stop);
    }
    
    /**
     * 
     * @param velocity rps
     * @return
     */
    public Command shootCommand(double velocity) {
        return runOnce(() -> {
            final VelocityVoltage request = new VelocityVoltage(velocity).withSlot(0);
            talonShoot.setControl(request);
        })
        .andThen(Commands.waitUntil(() -> Util.epsilonEquals(talonShoot.getVelocity().getValueAsDouble(), velocity, 1))) //?maybe tune epi
        .andThen(runOnce(() -> setSpeed(0.5)))
        .andThen(Commands.waitSeconds(1))
        .finallyDo(this::stop);
    }
}
