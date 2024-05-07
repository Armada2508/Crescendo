package frc.robot.subsystems;

import static frc.robot.lib.logging.NTLogger.log;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Pneumatics;

public class PneumaticsSubsystem extends SubsystemBase { 

    private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    private final Solenoid leftPiston = new Solenoid(PneumaticsModuleType.CTREPCM, Pneumatics.leftBottomChannel);
    private final Solenoid rightPiston = new Solenoid(PneumaticsModuleType.CTREPCM, Pneumatics.rightBottomChannel);
    private final Solenoid compressorFan = new Solenoid(PneumaticsModuleType.CTREPCM, Pneumatics.compressorFanChannel);

    public PneumaticsSubsystem() {
        compressor.enableDigital();
        compressorFan.set(true);
    }

    @Override
    public void periodic() {
        logPneumatics();
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
 
    private void logPneumatics() {
        log(this, "Left Piston Extended", leftPiston.get());
        log(this, "Right Piston Extended", rightPiston.get());
        log(this, "Compressor Status", compressor.isEnabled());
        log(this, "Compressor Current (A)", compressor.getCurrent());
        log(this, "Pressure Switch Tripped", compressor.getPressureSwitchValue());
        log(this, "Subsystem", this);
    }

}
