package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Pneumatics;
import frc.robot.lib.logging.Loggable;

public class PneumaticsSubsystem extends SubsystemBase implements Loggable {

    private final DoubleSolenoid leftPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Pneumatics.leftForwardChannel, Pneumatics.leftReverseChannel);
    private final DoubleSolenoid rightPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Pneumatics.rightForwardChannel, Pneumatics.rightReverseChannel);
    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

    public Command extend() {
        return runOnce(() -> {
            leftPiston.set(Value.kForward);
            rightPiston.set(Value.kForward);
        });
    }

    public Command retract() {
        return runOnce(() -> {
            leftPiston.set(Value.kReverse);
            rightPiston.set(Value.kReverse);
        });
    }

    public Command enableCompressor() {
        return runOnce(() -> compressor.enableDigital());
        
    }

    public Command disableCompressor() {
        return runOnce(() -> compressor.disable());
    }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        map.put("Left piston status", leftPiston.get());
        map.put("Right piston status", rightPiston.get());
        map.put("Compressor status", compressor.isEnabled());
        map.put("Compressor Current", compressor.getCurrent());
        return map;
    }
}
