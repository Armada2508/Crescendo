package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Pneumatics;
import frc.robot.lib.logging.Loggable;
import frc.robot.lib.logging.NTLogger;

public class PneumaticsSubsystem extends SubsystemBase implements Loggable { 

    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    private final Solenoid leftPiston = new Solenoid(PneumaticsModuleType.CTREPCM, Pneumatics.leftBottomChannel);
    private final Solenoid rightPiston = new Solenoid(PneumaticsModuleType.CTREPCM, Pneumatics.rightBottomChannel);

    public PneumaticsSubsystem() {
        NTLogger.register(this);
        compressor.enableDigital();
    }

    public Command extend() {
        return runOnce(() -> {
            leftPiston.set(true);
            rightPiston.set(true);
        }).withName("Extend Piston");
    }

    public Command retract() {
        return runOnce(() -> {
            leftPiston.set(false);
            rightPiston.set(false);
        }).withName("Retract Piston");
    }

    public Command enableCompressor() {
        return Commands.runOnce(() -> compressor.enableDigital()).withName("Enable Compressor");
    }

    public Command disableCompressor() {
        return Commands.runOnce(() -> compressor.disable()).withName("Disable Compressor");
    }

    public boolean isExtended() {
        return leftPiston.get() && rightPiston.get();
    }

    public void stop() {
        leftPiston.set(false);
        rightPiston.set(false);
    }
 
    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        map.put("Left Piston Extended", leftPiston.get());
        map.put("Right Piston Extended", rightPiston.get());
        map.put("Compressor Status", compressor.isEnabled());
        map.put("Compressor Current (A)", compressor.getCurrent());
        map.put("Pressure Switch Tripped", compressor.getPressureSwitchValue());
        NTLogger.putSubsystemLog(this, map);
        return map;
    }
}
