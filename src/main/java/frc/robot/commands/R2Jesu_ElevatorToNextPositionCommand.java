// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.R2Jesu_ElevatorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An R2Jesu_Elevator command that uses an R2Jesu_Elevatorer subsystem. */
public class R2Jesu_ElevatorToNextPositionCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final R2Jesu_ElevatorSubsystem m_subsystem;
  private boolean m_finish=false;

  /**
   * Creates a new R2Jesu_ElevatorCommand.
   * 
   * @param subsystem The subsystem used by this command.
   */
  public R2Jesu_ElevatorToNextPositionCommand(R2Jesu_ElevatorSubsystem subsystem) {
    m_subsystem = subsystem;
    m_finish=false;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.gotoNextPostition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_subsystem.targetiscurrent()) {
      m_finish=true;
    }
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
