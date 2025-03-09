// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.R2Jesu_ElevatorToPositionCommand;
import frc.robot.commands.R2Jesu_ElevatorToNextPositionCommand;
import frc.robot.commands.R2Jesu_ElevatorToPriorPositionCommand;
import frc.robot.subsystems.R2Jesu_ElevatorSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.commands.R2Jesu_AlignToTagCommand;
import frc.robot.subsystems.R2Jesu_AlgaeSubsystem;
import frc.robot.commands.R2Jesu_AlgaeIngestCommand;
import frc.robot.commands.R2Jesu_AlgaeRegurgitateCommand;
import frc.robot.commands.R2Jesu_DropCoralChuteCommand;
import frc.robot.commands.R2Jesu_AlgaeRaiseCommand;
import frc.robot.commands.R2Jesu_AlgaeLowerCommand;
import frc.robot.subsystems.R2Jesu_CoralSubsystem;
import frc.robot.commands.R2Jesu_ReleaseCoralCommand;
import frc.robot.subsystems.R2Jesu_HangerSubsystem;
import frc.robot.commands.R2Jesu_HangCommand;
import frc.robot.commands.R2Jesu_ReleaseHangerCommand;

public class RobotContainer {

   // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final         CommandXboxController driver2Xbox = new CommandXboxController(1);
  final         CommandJoystick buttonBoard = new CommandJoystick(2);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve"));
  private final R2Jesu_ElevatorSubsystem m_R2Jesu_ElevatorSubsystem = new R2Jesu_ElevatorSubsystem();
  private final R2Jesu_AlgaeSubsystem m_R2Jesu_AlgaeSubsystem = new R2Jesu_AlgaeSubsystem();
  private final R2Jesu_CoralSubsystem m_R2Jesu_CoralSubsystem = new R2Jesu_CoralSubsystem();
  private final R2Jesu_HangerSubsystem m_R2Jesu_HangerSubsystem = new R2Jesu_HangerSubsystem();

  public RobotContainer() {
    m_R2Jesu_AlgaeSubsystem.resetAlgaeEncoder();
    m_R2Jesu_ElevatorSubsystem.resetElevatorEncoder();
    configureBindings();
  }

  private void configureBindings() {
    drivebase.setDefaultCommand(
      drivebase.driveCommand(() -> driverXbox.getRightY(),
        () ->  driverXbox.getRightX(),
        () -> driverXbox.getLeftX()));

    driver2Xbox.start().onTrue(new SequentialCommandGroup(new R2Jesu_ReleaseHangerCommand(m_R2Jesu_HangerSubsystem), new R2Jesu_DropCoralChuteCommand(m_R2Jesu_CoralSubsystem)));
    driverXbox.leftTrigger().whileTrue(new R2Jesu_HangCommand(m_R2Jesu_HangerSubsystem));
    driver2Xbox.povUp().onTrue(new R2Jesu_ElevatorToNextPositionCommand(m_R2Jesu_ElevatorSubsystem));
    driver2Xbox.povDown().onTrue(new R2Jesu_ElevatorToPriorPositionCommand(m_R2Jesu_ElevatorSubsystem));
    driver2Xbox.leftTrigger().whileTrue(new R2Jesu_AlgaeIngestCommand(m_R2Jesu_AlgaeSubsystem));
    driver2Xbox.button(5).whileTrue(new R2Jesu_AlgaeRegurgitateCommand(m_R2Jesu_AlgaeSubsystem));
    driver2Xbox.rightTrigger().whileTrue(new R2Jesu_AlgaeLowerCommand(m_R2Jesu_AlgaeSubsystem));
    driver2Xbox.button(6).whileTrue(new R2Jesu_AlgaeRaiseCommand(m_R2Jesu_AlgaeSubsystem));
    driver2Xbox.button(2).onTrue(new R2Jesu_ReleaseCoralCommand(m_R2Jesu_CoralSubsystem));
    buttonBoard.button(1).onTrue(new SequentialCommandGroup(new R2Jesu_AlignToTagCommand(drivebase, true), new R2Jesu_ElevatorToPositionCommand(m_R2Jesu_ElevatorSubsystem, 3),
      new R2Jesu_ReleaseCoralCommand(m_R2Jesu_CoralSubsystem), new R2Jesu_ElevatorToPositionCommand(m_R2Jesu_ElevatorSubsystem, 0)));
    buttonBoard.button(2).onTrue(new SequentialCommandGroup(new R2Jesu_AlignToTagCommand(drivebase, false), new R2Jesu_ElevatorToPositionCommand(m_R2Jesu_ElevatorSubsystem, 3),
      new R2Jesu_ReleaseCoralCommand(m_R2Jesu_CoralSubsystem), new R2Jesu_ElevatorToPositionCommand(m_R2Jesu_ElevatorSubsystem, 0)));
    buttonBoard.button(3).onTrue(new SequentialCommandGroup(new R2Jesu_AlignToTagCommand(drivebase, true), new R2Jesu_ElevatorToPositionCommand(m_R2Jesu_ElevatorSubsystem, 2),
      new R2Jesu_ReleaseCoralCommand(m_R2Jesu_CoralSubsystem), new R2Jesu_ElevatorToPositionCommand(m_R2Jesu_ElevatorSubsystem, 0)));
    buttonBoard.button(4).onTrue(new SequentialCommandGroup(new R2Jesu_AlignToTagCommand(drivebase, false), new R2Jesu_ElevatorToPositionCommand(m_R2Jesu_ElevatorSubsystem, 2),
      new R2Jesu_ReleaseCoralCommand(m_R2Jesu_CoralSubsystem), new R2Jesu_ElevatorToPositionCommand(m_R2Jesu_ElevatorSubsystem, 0)));
    buttonBoard.button(5).onTrue(new SequentialCommandGroup(new R2Jesu_AlignToTagCommand(drivebase, true), new R2Jesu_ElevatorToPositionCommand(m_R2Jesu_ElevatorSubsystem, 1),
      new R2Jesu_ReleaseCoralCommand(m_R2Jesu_CoralSubsystem), new R2Jesu_ElevatorToPositionCommand(m_R2Jesu_ElevatorSubsystem, 0)));
    buttonBoard.button(6).onTrue(new SequentialCommandGroup(new R2Jesu_AlignToTagCommand(drivebase, false),   new R2Jesu_ElevatorToPositionCommand(m_R2Jesu_ElevatorSubsystem, 1),
      new R2Jesu_ReleaseCoralCommand(m_R2Jesu_CoralSubsystem), new R2Jesu_ElevatorToPositionCommand(m_R2Jesu_ElevatorSubsystem, 0)));
    buttonBoard.button(7).onTrue(new R2Jesu_ReleaseCoralCommand(m_R2Jesu_CoralSubsystem));

    //temp for testing
    driver2Xbox.button(3).onTrue(new R2Jesu_AlignToTagCommand(drivebase, true));
    driver2Xbox.button(4).onTrue(new R2Jesu_AlignToTagCommand(drivebase, false));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
