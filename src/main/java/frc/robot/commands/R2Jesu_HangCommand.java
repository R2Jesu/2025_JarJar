// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.R2Jesu_HangerSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An R2Jesu_Hanger command that uses an R2Jesu_Hangerer subsystem. */
public class R2Jesu_HangCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private boolean m_finish;

  private final R2Jesu_HangerSubsystem m_subsystem;

  /**
   * Creates a new R2Jesu_HangerCommand.
   * 
   * @param subsystem The subsystem used by this command.
   */
  public R2Jesu_HangCommand(R2Jesu_HangerSubsystem subsystem) {
    m_subsystem = subsystem; 
    m_finish=false;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_subsystem.isHangerReleased()) {
      m_subsystem.hang(1.0);
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.hang(0);
    //m_finish=true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finish;
  }
}
