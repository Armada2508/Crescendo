package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.logging.Loggable;

public class PneumaticsSubsystem extends SubsystemBase implements Loggable {

    private final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0); //! find channels
    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM); //add id's for compressors

    private PneumaticsSubsystem() {}

    public final Command extend() {
        if (doubleSolenoid.get() == Value.kReverse) {
            return runOnce(() -> doubleSolenoid.toggle());
        } return runOnce(() -> {}); //! fix / verify
    }

    public final Command retract() {
        if (doubleSolenoid.get() == Value.kForward) {
            return runOnce(() -> doubleSolenoid.toggle());
        } return runOnce(() -> {}); //! fix / verify
    }

    public final Command enableCompressor() {
        return runOnce(() -> compressor.enableDigital());
        
    }

    public final Command disableCompressor() {
        return runOnce(() -> compressor.disable());
    }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        map.put("Solenoid status", doubleSolenoid.get());
        map.put("Compressor status", compressor.isEnabled());
        map.put("Compressor pressure", compressor.getPressure());
        map.put("Compressor Current", compressor.getCurrent());
        return map;
    }
}
