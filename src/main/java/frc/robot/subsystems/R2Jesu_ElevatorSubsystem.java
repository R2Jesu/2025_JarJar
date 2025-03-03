// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.Math;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;



public class R2Jesu_ElevatorSubsystem extends SubsystemBase {
  private SparkMax elevator1 = new SparkMax(9, MotorType.kBrushless);
  private SparkMax elevator2 = new SparkMax(10, MotorType.kBrushless);
  private Encoder elevatorEncoder = new Encoder(1,2, true, CounterBase.EncodingType.k4X);
  private static int currentPosition=0;
  private int targetPosition=0;
  private double elevatorStops[] = {0.0, 3.0, 11.0, 25};
  private PIDController m_elevatorController = new PIDController(.15, 0.0, 0.0, 0.01); //p 1.5
  private PIDController m_elevatorDownController = new PIDController(.05, 0.0, 0.0, 0.01); //p 1.5
  private double pidOutput;
  private double downpidOutput;
  private DigitalInput elevatorLimit = new DigitalInput(8);
  
  
  /** Creates a new R2Jesu_ElevatorSubsystem. */

  /** Here we will eventuall put the motor defintions that we need to control to raise and lower the elevator */
 


  /**
   * R2Jesu_Elevator command factory method.
   *
   * @return a command
   */
  public Command R2Jesu_ElevatorMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An R2Jesu_Elevator method querying a boolean state of the subsystem (for R2Jesu_Elevator, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean R2Jesu_ElevatorCondition() {
    // Query some boolean state, such as a digital sensor.
    return true;
  }

  public void moveElevator(double speed) {

    /* raise the elevator which will set the motor in the proper direction to raise
     * this may need to take in a speed and the PID logic for a raise to x level with
     * the PID slowing the speed on approach
     */
    if (speed > .25)
    {
      speed = .25;
    }
    if (speed < -.15)
    {
      speed = -.15;
    }

    elevator1.set(speed);
    elevator2.set(-speed);
  } 

  public void raiseElevator(double speed) {

  /* raise the elevator which will set the motor in the proper direction to raise
   * this may need to take in a speed and the PID logic for a raise to x level with
   * the PID slowing the speed on approach
   */
  elevator1.set(speed);
  elevator2.set(-speed);
}  

public void lowerElevator(double speed) {
  /* lower the elevator which will set the motor in the proper direction to lower
   * this may need to take in a speed and the PID logic for a lower to x level with
   * the PID slowing the speed on approach
   */
  if (speed < 0) {
    speed = 0;
  }
  elevator1.set(-speed);
  elevator2.set(speed);

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

  if (targetPosition < elevatorStops.length - 1)
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

  if (targetPosition > 0)
  {
     targetPosition=targetPosition - 1;
  }

} 

public void resetElevatorEncoder() {
  /* lower the algae which will set the motor in the proper direction to lower
   * this may need to take in a speed and the PID logic for a lower to x level with
   * the PID slowing the speed on approach
   */
  elevatorEncoder.reset();
  elevatorEncoder.setDistancePerPulse(1.0 / 2048.0 * 2.0 * 3.14159 * (1.76 / 2));

}

public static int getElevatorLevel() {
  /* lower the algae which will set the motor in the proper direction to lower
   * this may need to take in a speed and the PID logic for a lower to x level with
   * the PID slowing the speed on approach
   */
  return currentPosition;

}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("periodic");

  // To tuen it off when needed
  //pidOutput=0;
  if (elevatorLimit.get())
  {
    downpidOutput = m_elevatorDownController.calculate(elevatorEncoder.getDistance(), elevatorStops[targetPosition]);
    pidOutput = m_elevatorController.calculate(elevatorEncoder.getDistance(), elevatorStops[targetPosition]); 
  }
  else
  {
    downpidOutput = 0;
    pidOutput = 0;
  
  }

  if (targetPosition < currentPosition)
  {
      this.moveElevator(downpidOutput);
  }
  else if (targetPosition > currentPosition)
  {
    this.moveElevator(pidOutput);
  }
  else
  {
    if (currentPosition == 0)
    {
      pidOutput = 0;
      resetElevatorEncoder();
    }
    this.moveElevator(pidOutput);
  } 

  if (Math.abs(elevatorEncoder.getDistance() - elevatorStops[targetPosition]) < 1)
  {
    currentPosition=targetPosition;
  }
    
    SmartDashboard.putNumber("encoderdistance", elevatorEncoder.getDistance());
    SmartDashboard.putNumber("currentPosition", currentPosition);
    SmartDashboard.putNumber("targetPosition", targetPosition);
    SmartDashboard.putNumber("pidOutput", pidOutput);
    SmartDashboard.putNumber("downpidOutput", downpidOutput);
    SmartDashboard.putNumber("targetDistance", elevatorStops[targetPosition]);
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
