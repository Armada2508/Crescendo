package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.logging.Loggable;
import frc.robot.lib.pneumatics.Piston;

public class PneumaticsSubsystem extends SubsystemBase implements Loggable {
    private final Piston leftPiston = new Piston(0, 0);
    private final Piston rightPiston = new Piston(0, 0);
    private final Compressor leftCompressor = new Compressor(PneumaticsModuleType.CTREPCM); //add id's for compressors
    private final Compressor rightCompressor = new Compressor(PneumaticsModuleType.CTREPCM); 

    private PneumaticsSubsystem() {}

    public final Command extend() {
        return runOnce(() -> {
            leftPiston.extend();
            rightPiston.extend();
        });
    }

    public final Command retract(Piston piston) {
        return runOnce(() -> {
            leftPiston.retract();
            rightPiston.retract();
        });
    }

    public final Command enableCompressor() {
        return runOnce(() -> {
            leftCompressor.enableDigital();
            rightCompressor.enableDigital();
        });
        
    }

    public final Command disableCompressor() {
        return runOnce(() -> {
            leftCompressor.disable();
            rightCompressor.disable();
        });
    }

    private final boolean getStatus (Piston piston) { //! Fix, return true if piston is extended, false if retracted
        return false;
    }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        map.put("Left piston status", getStatus(leftPiston));
        map.put("Right piston status", getStatus(rightPiston));
        map.put("Left compressor status", leftCompressor.isEnabled());
        map.put("Right compressor status", rightCompressor.isEnabled());
        map.put("Left compressor pressure", leftCompressor.getPressure());
        map.put("Right compressor pressure", rightCompressor.getPressure());
        return map;
    }
}
