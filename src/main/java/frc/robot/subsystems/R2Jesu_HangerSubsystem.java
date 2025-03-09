// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class R2Jesu_HangerSubsystem extends SubsystemBase {
  private SparkMax hanger1 = new SparkMax(15, MotorType.kBrushed);
  private static boolean hangerReleased = false;
 
  /** Creates a new R2Jesu_HangerSubsystem. */

  /** Here we will eventuall put the motor defintions that we need to control to raise and lower the hanger */
 


  /**
   * R2Jesu_Hanger command factory method.
   *
   * @return a command
   */
  public Command R2Jesu_HangerMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An R2Jesu_Hanger method querying a boolean state of the subsystem (for R2Jesu_Hanger, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean R2Jesu_HangerCondition() {
    // Query some boolean state, such as a digital sensor.
    return true;
  }

  public void hang(double speed) {
    hanger1.set(speed);
  } 

  public void setReleased() {
    hangerReleased = true;
  }

  public void releaseHanger() {
    //Set servo to 0
  }

  public boolean isHangerReleased() {
    return hangerReleased;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //SmartDashboard.putNumber("encoderdistance", hangerEncoder.getDistance());
   
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
