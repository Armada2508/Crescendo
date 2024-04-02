package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Pneumatics;
import frc.robot.lib.logging.Loggable;
import frc.robot.lib.logging.NTLogger;
import frc.robot.lib.pneumatics.Piston;

public class PneumaticsSubsystem extends SubsystemBase implements Loggable { 

    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    private final Piston leftPiston = new Piston(Pneumatics.leftReverseChannel, Pneumatics.leftForwardChannel);
    private final Piston rightPiston = new Piston(Pneumatics.rightReverseChannel, Pneumatics.rightForwardChannel);
    private final Solenoid compressorFan = new Solenoid(PneumaticsModuleType.CTREPCM, Pneumatics.compressorFanChannel);

    public PneumaticsSubsystem() {
        NTLogger.register(this);
        compressor.enableDigital();
        compressorFan.set(true);
    }

    public Command extend() {
        return runOnce(() -> {
            leftPiston.extend();
            rightPiston.extend();
        });
    }

    public Command retract() {
        return runOnce(() -> {
            leftPiston.retract();
            rightPiston.retract();
        });
    }

    public Command enableCompressor() {
        return runOnce(() -> {
            compressor.enableDigital();
        });
    }

    public Command disableCompressor() {
        return runOnce(() -> compressor.disable());
    }

    public boolean isExtended() {
        return leftPiston.isExtended() && rightPiston.isExtended();
    }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        map.put("Left Piston Extended", leftPiston.isExtended());
        map.put("Right Piston Extended", rightPiston.isExtended());
        map.put("Compressor Status", compressor.isEnabled());
        map.put("Compressor Current (A)", compressor.getCurrent());
        map.put("Pressure Switch Tripped", compressor.getPressureSwitchValue());
        return map;
    }
}
