// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.R2Jesu_CoralSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An R2Jesu_Coral command that uses an R2Jesu_Coraler subsystem. */
public class R2Jesu_WaitForCoralCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private boolean m_finish;
  private Timer overallTimer;
  private final R2Jesu_CoralSubsystem m_subsystem;

  /**
   * Creates a new R2Jesu_CoralCommand.
   * 
   * @param subsystem The subsystem used by this command.
   */
  public R2Jesu_WaitForCoralCommand(R2Jesu_CoralSubsystem subsystem) {
    m_subsystem = subsystem; 
    m_finish=false;


    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.overallTimer = new Timer();
    this.overallTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_subsystem.haveCoral()) {
      m_finish=true;
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finish || overallTimer.hasElapsed(3.0);
  }
}
