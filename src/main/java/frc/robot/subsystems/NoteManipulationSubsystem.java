package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Millimeters;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.lib.util.Util;

public class NoteManipulationSubsystem extends SubsystemBase {

    private final TalonFX talonIntake = new TalonFX(Intake.intakeID);
    private final TalonFX talonShoot = new TalonFX(Intake.shooterID); 
    private final TimeOfFlight timeOfFlight = new TimeOfFlight(Intake.timeOfFlightID);

    public NoteManipulationSubsystem() {
        configTalons();
    }

    private void configTalons() {
        Util.factoryResetTalons(talonIntake, talonShoot);
        Util.coastMode(talonIntake, talonShoot);
        talonShoot.getConfigurator().apply(Intake.slot0Config);
    }

    public void setIntakeSpeed(double speed) {
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
        return Util.inRange(timeOfFlight.getRange(), Intake.noteDetectionRange.in(Millimeters));
    }

    public Command intakeCommand() {
        return runOnce(() -> {
            setIntakeSpeed(0.5); //! Constant
        })
        .andThen(Commands.waitUntil(() -> isSensorTripped()))
        .andThen(Commands.waitSeconds(1)) //! Constant
        .finallyDo(this::stop);
    }
    
    /**
     * 
     * @param velocity rotations per second
     * @return
     */
    public Command shootCommand(double velocity) {
        return runOnce(() -> {
            final VelocityVoltage request = new VelocityVoltage(velocity);
            talonShoot.setControl(request);
        })
        .andThen(Commands.waitUntil(() -> Util.epsilonEquals(talonShoot.getVelocity().getValueAsDouble(), velocity, 1))) //! Constant
        .andThen(runOnce(() -> setIntakeSpeed(0.5))) //! Constant
        .andThen(Commands.waitSeconds(1)) //! Constant
        .finallyDo(this::stop);
    }

}
