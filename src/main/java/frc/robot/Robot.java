// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.lib.logging.NTLogger.log;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.lib.logging.NTLogger;

public class Robot extends TimedRobot {
    
    private RobotContainer robotContainer;
    private BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();
    
    @Override
    public void robotInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
        NTLogger.initDataLogger();
        robotContainer = new RobotContainer();
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        NTLogger.logDriverStation();
        logRobot();
    }

    @Override
    public void autonomousInit() {
        robotContainer.getAutonomousCommand().schedule();
    }

    @Override
    public void teleopInit() {
        robotContainer.stopEverything();
    }
    
    @Override
    public void disabledInit() {
        robotContainer.stopEverything();
    }

    private void logRobot() {
        log(this, "Battery Voltage (V)", RobotController.getBatteryVoltage());
        log(this, "RIO Voltage (V)", RobotController.getInputVoltage());
        log(this, "RIO Current (A)", RobotController.getInputCurrent());
        log(this, "CAN Bus Utilization %", RobotController.getCANStatus().percentBusUtilization);
        log(this, "Accelerometer X (g)", accelerometer.getX());
        log(this, "Accelerometer Y (g)", accelerometer.getY());
        log(this, "Accelerometer Z (g)", accelerometer.getZ());
    }

    public static boolean onRedAlliance() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    }

    @Override
    public void simulationPeriodic() {}
    
}
