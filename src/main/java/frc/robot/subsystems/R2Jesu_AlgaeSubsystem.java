// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class R2Jesu_AlgaeSubsystem extends SubsystemBase {
  /** Creates a new R2Jesu_ElevatorSubsystem. */
  private Encoder algaeArmEncoder = new Encoder(1,2, true, CounterBase.EncodingType.k4X);

  /** Here we will eventuall put the motor defintions that we need to control to raise and lower the elevator */
 


  /**
   * R2Jesu_Elevator command factory method.
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
   * An R2Jesu_Elevator method querying a boolean state of the subsystem (for R2Jesu_Elevator, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean R2Jesu_AlgaeCondition() {
    // Query some boolean state, such as a digital sensor.
    return true;
  }

  public void raiseAlgaeArm() {
  /* raise the arm which will set the motor in the proper direction to raise
   * this may need to take in a speed and the PID logic for a raise to x level with
   * the PID slowing the speed on approach
   */
}  

public void lowerAlgaeArm() {
  /* lower the arm which will set the motor in the proper direction to lower
   * this may need to take in a speed and the PID logic for a lower to x level with
   * the PID slowing the speed on approach
   */
} 

public void gotoPostition() {
  /* This may take in a position on the reef representing stowed, or a couple levels matching algae on the reef
   * Then it will determine our current position and raise or lower to go to requested
   * This is the one that would read the degrees and determine when to stop the call
   * Will likely use a PID to set speed
  */
} 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
