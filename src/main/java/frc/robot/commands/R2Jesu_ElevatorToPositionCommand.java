// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.R2Jesu_ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An R2Jesu_Elevator command that uses an R2Jesu_Elevatorer subsystem. */
public class R2Jesu_ElevatorToPositionCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private int m_position;
  private boolean m_finish;

  private final R2Jesu_ElevatorSubsystem m_subsystem;

  /**
   * Creates a new R2Jesu_ElevatorCommand.
   * 
   * @param subsystem The subsystem used by this command.
   */
  public R2Jesu_ElevatorToPositionCommand(R2Jesu_ElevatorSubsystem subsystem, int pos) {
    m_subsystem = subsystem; 
    m_position=pos;
    m_finish=false;


    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.gotoPostition(m_position);
    m_finish=true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finish;
  }
}
