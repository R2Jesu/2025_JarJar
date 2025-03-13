// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import au.grapplerobotics.CanBridge;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    CanBridge.runTCP();
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
/*      m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Shoot leave", kShootLeaveAuto);
    m_chooser.addOption("Shoot leave shoot", kShootLeaveShootAuto);
    m_chooser.addOption("Shoot leave off center", kShootLeaveOffCenterAuto);
    m_chooser.addOption("Shoot leave shoot off center", kShootLeaveShootOffCenterAuto); */
  
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    SmartDashboard.putString("Choice", m_autonomousCommand.toString());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    SmartDashboard.putString("Choice", m_autonomousCommand.toString());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    m_robotContainer.drivebase.resetOurOdometry();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
