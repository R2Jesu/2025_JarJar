// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.Math;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class R2Jesu_AlgaeSubsystem extends SubsystemBase {

  private Encoder algaeEncoder = new Encoder(5,6, true, CounterBase.EncodingType.k4X);
  private TalonSRX algae1 = new TalonSRX(11);
  private int currentPosition=0;
  private int targetPosition=0;
  private double algaeStops[] = {0.0, 50.0, 100.0, 150.0};
  private PIDController m_algaeController = new PIDController(.005, 0.0, 0.0, 0.01); //p 1.5
  private PIDController m_algaeDownController = new PIDController(.005, 0.0, 0.0, 0.01); //p 1.5
  private double pidOutput;
  private double downpidOutput;
  
  /** Creates a new R2Jesu_AlgaeSubsystem. */

  /** Here we will eventuall put the motor defintions that we need to control to raise and lower the algae */
 


  /**
   * R2Jesu_Algae command factory method.
   *
   * @return a command
   */
  public Command R2Jesu_AlgaeMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An R2Jesu_Algae method querying a boolean state of the subsystem (for R2Jesu_Algae, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean R2Jesu_AlgaeCondition() {
    // Query some boolean state, such as a digital sensor.
    return true;
  }

  public void moveAlgae(double speed) {

    /* raise the algae which will set the motor in the proper direction to raise
     * this may need to take in a speed and the PID logic for a raise to x level with
     * the PID slowing the speed on approach
     */
    algae1.set(ControlMode.PercentOutput, speed);
  }

  public void raiseAlgae(double speed) {

  /* raise the algae which will set the motor in the proper direction to raise
   * this may need to take in a speed and the PID logic for a raise to x level with
   * the PID slowing the speed on approach
   */
  algae1.set(ControlMode.PercentOutput, speed);
}  

public void lowerAlgae(double speed) {
  /* lower the algae which will set the motor in the proper direction to lower
   * this may need to take in a speed and the PID logic for a lower to x level with
   * the PID slowing the speed on approach
   */
  algae1.set(ControlMode.PercentOutput, -speed);

} 

public void gotoPostition(int position) {
  /* This may take in a position on the reef representing a coral height
   * Then it will determine our current position and raise or lower to go to requested
   * This is the one that would read the pulses and determine when to stop the call
   * Will likely use a PID to set speed
  */
  targetPosition=position;

} 

public void gotoNextPostition() {
  /* This may take in a position on the reef representing a coral height
   * Then it will determine our current position and raise or lower to go to requested
   * This is the one that would read the pulses and determine when to stop the call
   * Will likely use a PID to set speed
  */
  System.out.println("next");
  if (targetPosition < algaeStops.length - 1)
  {
     targetPosition=targetPosition + 1;
  }

} 

public void gotoPriorPostition() {
  /* This may take in a position on the reef representing a coral height
   * Then it will determine our current position and raise or lower to go to requested
   * This is the one that would read the pulses and determine when to stop the call
   * Will likely use a PID to set speed
  */
  System.out.println("prior");
  if (targetPosition > 0)
  {
     targetPosition=targetPosition - 1;
  }

} 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("periodic");

  // To tuen it off when needed
  //pidOutput=0;
  downpidOutput = -(m_algaeDownController.calculate(algaeEncoder.getDistance(), algaeStops[targetPosition]));
  pidOutput = -(m_algaeController.calculate(algaeEncoder.getDistance(), algaeStops[targetPosition])); 


  if (targetPosition < currentPosition)
  {
      this.moveAlgae(downpidOutput);
  }
  else if (targetPosition > currentPosition)
  {
    this.moveAlgae(pidOutput);
  }
  else
  {
    if (currentPosition == 0)
    {
      pidOutput = 0; 
      algaeEncoder.reset();
    }
    this.moveAlgae(pidOutput);
  } 

  if (Math.abs(algaeEncoder.getDistance() - algaeStops[targetPosition]) < 25)
  {
    currentPosition=targetPosition;
  }
    
    SmartDashboard.putNumber("Algaeencoderdistance", algaeEncoder.getDistance());
    SmartDashboard.putNumber("AlgaecurrentPosition", currentPosition);
    SmartDashboard.putNumber("AlgaetargetPosition", targetPosition);
    SmartDashboard.putNumber("AlgaepidOutput", pidOutput);
    SmartDashboard.putNumber("AlgaetargetDistance", algaeStops[targetPosition]);
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
