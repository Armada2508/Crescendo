// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.Field;
import frc.robot.lib.logging.NTLogger;

public class Robot extends TimedRobot {
    
    private RobotContainer robotContainer;
    
    @Override
    public void robotInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
        robotContainer = new RobotContainer();
        NTLogger.initDataLogger();
        SmartDashboard.putData(Field.simulatedField);
    }
    
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        NTLogger.log();
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
