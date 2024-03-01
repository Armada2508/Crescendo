package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
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
        talonFollow.setControl(new StrictFollower(talon.getDeviceID()));
    }

    public Command setClimbVoltage(Measure<Voltage> volts) {
        return runOnce(() -> talon.setControl(new DutyCycleOut(volts.in(Volts))));
    }

    public void stop() {
        talon.setControl(new NeutralOut());
    }

    public Command climbCommand(double volts) {
        return setClimbVoltage(Volts.of(volts)) //extend
        .andThen(setClimbVoltage(Volts.of(-volts))); //pull up
       // .andThen(Commands.waitSeconds(2));
    }

    public Command unClimbCommand(double volts) {
        return setClimbVoltage(Volts.of(-volts)); //drop down
    }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        throw new UnsupportedOperationException("Unimplemented method 'log'");
    }
}


/*
2 motors that controls a winch, 10" acutation distance
mirrored

Methods:
Constructor
Config Talons
Set Voltage
Stop
Climb Command
Unclimb Command (?)
Log
 */