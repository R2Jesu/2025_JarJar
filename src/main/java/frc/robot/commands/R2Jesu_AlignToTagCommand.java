// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utilities.LimelightHelpers;

/** An SwerveSubsystem command that uses an SwerveSubsystem subsystem. */
public class R2Jesu_AlignToTagCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private boolean sideL;
  private Timer dontSeeTagTimer, stopTimer, overallTimer;
  private PIDController xControl = new PIDController(1.5, 0, 0);
  private PIDController yControl = new PIDController(2, 0, 0);  
  private PIDController zControl = new PIDController(.058, 0, .0);

  private final SwerveSubsystem m_subsystem;

  /**
   * Creates a new SwerveCommand.
   * 
   * @param subsystem The subsystem used by this command.
   */
  public R2Jesu_AlignToTagCommand(SwerveSubsystem subsystem, boolean direction) {
    m_subsystem = subsystem; 
    sideL=direction;

    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();
    this.overallTimer = new Timer();
    this.overallTimer.start();
    zControl.setSetpoint(0.0);
    zControl.setTolerance(.2);

    if (sideL) {
      yControl.setSetpoint(-.19);
    }
    else {
      yControl.setSetpoint(.19);
    }
    yControl.setTolerance(.1);

    xControl.setSetpoint(0.0);
    xControl.setTolerance(.2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimelightHelpers.getFiducialID("limelight") != 0) {
      dontSeeTagTimer.reset();
      double[] positions = LimelightHelpers.getCameraPose_TargetSpace("limelight");

      double xSpeed = xControl.calculate(positions[1]);
      SmartDashboard.putNumber("xspeed", xSpeed);
      double ySpeed = -yControl.calculate(positions[0]);
      SmartDashboard.putNumber("yspeed", ySpeed);
      double rotValue = -zControl.calculate(positions[4]);
      SmartDashboard.putNumber("zspeed", rotValue);
      SmartDashboard.putNumber("tx:", LimelightHelpers.getTX("limelight"));
      

      m_subsystem.drive(new Translation2d(m_subsystem.distInIn > 9 ? xSpeed : 0, ySpeed), rotValue, false);
      //m_subsystem.drive(new Translation2d(yControl.getError() < 0.2 ? xSpeed : 0, ySpeed), rotValue, false);
      //m_subsystem.drive(new Translation2d(xControl.atSetpoint() ? 0 : xSpeed, ySpeed), rotValue, false);
      //m_subsystem.drive(new Translation2d(0, ySpeed), rotValue, false);


      if (!zControl.atSetpoint() ||
          !yControl.atSetpoint() ||
          !xControl.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      m_subsystem.drive(new Translation2d(), 0, false);
      System.out.println("hit set");
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.drive(new Translation2d(), 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.dontSeeTagTimer.hasElapsed(1.0) ||
        stopTimer.hasElapsed(0.3) || overallTimer.hasElapsed(3.0);
  }
}
