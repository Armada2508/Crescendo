package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
    private final DoubleSolenoid leftPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Pneumatics.leftForwardChannel, Pneumatics.leftReverseChannel);
    // private final DoubleSolenoid rightPiston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Pneumatics.rightForwardChannel, Pneumatics.rightReverseChannel);
    private final Piston rightPiston = new Piston(Pneumatics.rightReverseChannel, Pneumatics.rightForwardChannel);
    private final Solenoid compressorFan = new Solenoid(PneumaticsModuleType.CTREPCM, Pneumatics.compressorFanChannel);

    public PneumaticsSubsystem() {
        NTLogger.register(this);
        compressor.disable();
        compressorFan.set(true);
    }

    public Command extend() {
        return runOnce(() -> {
            leftPiston.set(Value.kForward);
            // rightPiston.set(Value.kForward);
            rightPiston.extend();
        });
    }

    public Command retract() {
        return runOnce(() -> {
            leftPiston.set(Value.kReverse);
            // rightPiston.set(Value.kReverse);
            rightPiston.retract();
        });
    }

    public Command enableCompressor() {
        return runOnce(() -> {
            compressor.enableDigital();
            compressorFan.set(true); //! remove me!
        });
    }

    public Command disableCompressor() {
        return runOnce(() -> compressor.disable());
    }

    public Command setFan(boolean val) { //! remove me!
        return runOnce(() ->compressorFan.set(val));
    }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        map.put("Left Piston Status", leftPiston.get());
        // map.put("Right Piston Status", rightPiston.get());
        map.put("Right Piston Extended", rightPiston.isExtended());
        map.put("Compressor Status", compressor.isEnabled());
        map.put("Compressor Current (A)", compressor.getCurrent());
        map.put("Pressure Switch Tripped", compressor.getPressureSwitchValue());
        return map;
    }
}
