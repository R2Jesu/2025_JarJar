// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.R2Jesu_ElevatorToPositionCommand;
import frc.robot.subsystems.R2Jesu_ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class RobotContainer {

   // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  private final R2Jesu_ElevatorSubsystem m_R2Jesu_ElevatorSubsystem = new R2Jesu_ElevatorSubsystem();
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drivebase.setDefaultCommand(
      drivebase.driveCommand(() -> -driverXbox.getRightY(),
        () ->  -driverXbox.getRightX(),
        () -> -driverXbox.getLeftX()));

    driverXbox.button(2).whileTrue(new R2Jesu_ElevatorToPositionCommand(m_R2Jesu_ElevatorSubsystem, 1 ));
    driverXbox.button(1).whileTrue(drivebase.aimAtTarget());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
