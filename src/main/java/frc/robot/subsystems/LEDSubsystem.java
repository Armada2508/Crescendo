package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants.LED;
import frc.robot.lib.led.LEDStrip;

public class LEDSubsystem {

    private static LEDStrip led = new LEDStrip(LED.ledPort, 10); //placeholder number, led on the bottom of the robot
    
    public LEDSubsystem() {

    }

    @Override
    public void periodic() {
        Measure<Time> matchTime = Seconds.of(DriverStation.getMatchTime());

        if (DriverStation.isAutonomous() == true) { //match is in autonomous
            autoStart();
        }
        if ((!DriverStation.isAutonomous() == true) && DriverStation.isTeleop() == true) { //match is not in autonomous and is in teleop
            teleopStart();
        }
        if (matchTime.gte(Seconds.of(130)) && DriverStation.isTeleop() == true) {
            endgameStart();
        }
    }

    public void autoStart() {
        Color color = new Color(255, 255, 255);
        led.band(3, color);
    }

    public void teleopStart() {
        Color color = new Color(255, 0, 255);
        led.band(3, color);
    }

    public void endgameStart() {
        Color color = new Color(0, 0, 255);
        led.band(3, color);
    }

    public void intakeNoteAnimation(IntakeShooterSubsystem intakeShooterSubsystem) {
        Color red = new Color(255, 0, 0);
        Color green = new Color(0, 255, 0);
        if (intakeShooterSubsystem.isSensorTripped() == true) {
            led.pulseCommand(red, green, 0.5)
            .end(true);
        }
    }
}
