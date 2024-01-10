package frc.robot.subsystems;

import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.util.Util;

import static frc.robot.Constants.Pivot.ID;
import static frc.robot.Constants.Pivot.FID;

public class PivotSubsystem {
    
    final TalonFX talon = new TalonFX(ID);
    final TalonFX talonF = new TalonFX(FID);

    public PivotSubsystem() {
        configureTalon();
    }

    public void configureTalon() { //update void if need to return something
        Util.factoryResetTalons(talon, talonF);
        talonF.setControl(new StrictFollower(talon.getDeviceID()));
    }

    public void setSpeed(double speed) {
        talon.set(speed);
    }

    public Command setAngleCommand(double theta) { 

        return null; //! Update when done (error is annoying to look at)
    }
}
