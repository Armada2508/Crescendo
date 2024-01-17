package frc.robot.subsystems;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;
import frc.robot.lib.util.Util;

public class IntakeSubsystem extends SubsystemBase{

    private final TalonFX talonIntake = new TalonFX(Intake.INID); // intaking / index
    private final TalonFX talonShoot = new TalonFX(Intake.SID); // shooting

    public IntakeSubsystem() {
        configTalons();
    }

    private void configTalons() {
        Util.factoryResetTalons(talonIntake, talonShoot);
        Util.coastMode(talonIntake, talonShoot);
        talonShoot.getConfigurator().apply(Intake.slot0Config);
    }

    public void setSpeed(double speed, TalonFX talon) {
        talon.set(speed);
    }

    public void stop() {
        talonIntake.setControl(new NeutralOut());
        talonShoot.setControl(new NeutralOut());
    }

    public boolean isSensorTripped() {
        //fill in once sensor details are finished
        return false;
    }

    public void intake() {
        setSpeed(0.5, talonIntake); //? possibly tune speed
        //sensor something
        setSpeed(0, talonIntake); 
    }

    public Command intakeCommand() {
        return runOnce(() -> {
            intake();
        })
        .finallyDo(this::stop);
    }

    public void shoot(double velocity) {
        //wait until velocity at set point
        setSpeed(0.5, talonIntake); //runs index motor, test speed //? change speed depending on target (amp or speaker)?
        Timer.delay(1); //! possibly dangerous, test
    }

    public Command shootCommand(double velocity) {
        return runOnce(() -> {
            shoot(velocity);
        })
        .finallyDo(this::stop);
    }
}
