// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Field;
import frc.robot.lib.logging.Loggable;
import frc.robot.lib.logging.NTLogger;

public class Robot extends TimedRobot implements Loggable {
    
    private RobotContainer robotContainer;
    
    @Override
    public void robotInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
        // NTLogger.initDataLogger();
        SmartDashboard.putData(Field.simulatedField);
        NTLogger.register(this);
        robotContainer = new RobotContainer();
        addPeriodic(NTLogger::log, kDefaultPeriod, 0);
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
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

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public Map<String, Object> log(Map<String, Object> map) {
        map.put("Battery Voltage (V)", RobotController.getBatteryVoltage());
        map.put("RIO Voltage (V)", RobotController.getInputVoltage());
        map.put("RIO Current (A)", RobotController.getInputCurrent());
        map.put("CAN Bus Utilization %", RobotController.getCANStatus().percentBusUtilization);
        return map;
    }

    public static boolean onRedAlliance() {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
    }

    @Override
    public void simulationPeriodic() {}
    
    @Override
    public void disabledPeriodic() {}
    
    @Override
    public void disabledExit() {}
    
    @Override
    public void autonomousPeriodic() {}
    
    @Override
    public void autonomousExit() {}
    
    @Override
    public void teleopPeriodic() {}
    
    @Override
    public void teleopExit() {}
    
    @Override
    public void testPeriodic() {}
    
    @Override
    public void testExit() {}
    
}
