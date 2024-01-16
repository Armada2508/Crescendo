package frc.robot.subsystems;

import static frc.robot.Constants.Intake.INID;
import static frc.robot.Constants.Intake.SID;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.lib.util.Util;

public class IntakeSubsystem extends SubsystemBase{

    final TalonFX talonIntake = new TalonFX(INID); // intaking
    final TalonFX talonShoot = new TalonFX(SID); //shooting

    public IntakeSubsystem() {
        configTalons();
    }

    private void configTalons() {
        Util.factoryResetTalons(talonIntake, talonShoot);
    }

    public void setSpeed(double speed, TalonFX talon) {
        talon.set(speed);
    }

    public void stop() {
        talonIntake.setControl(new NeutralOut());
        talonShoot.setControl(new NeutralOut());
    }

    public Boolean isSensorTripped() {
        //fill in once sensor details are finished
        return false;
    }

    public void intake() {
        setSpeed(1, talonIntake); //? possibly tune speed
        while (isSensorTripped() == false) { //check w/ brock to see if bad
            if (isSensorTripped() == true) {
                setSpeed(0, talonIntake);
            }
        }
    }

    public Command intakeCommand() {
        return runOnce(() -> {
            intake();
        })
        .finallyDo(this::stop);
    }

    public void shoot() {

    }

    public Command shootCommand() {
        return runOnce(() -> {

        })
        .andThen() // if needed
        .finallyDo(this::stop);
    }
}
