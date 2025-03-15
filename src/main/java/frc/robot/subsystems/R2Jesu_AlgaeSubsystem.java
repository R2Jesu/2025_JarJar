// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class R2Jesu_AlgaeSubsystem extends SubsystemBase {

  //private Encoder algaeEncoder = new Encoder(5,6, true, CounterBase.EncodingType.k4X);
  private TalonSRX algae1 = new TalonSRX(11);
  private TalonSRX algaeWheels = new TalonSRX(12);
  double encoderMax=300.0;
  double encoderMin=10.0;
  int thiswaycount=0;
  int thatwaycount=0;
  
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

    //This motor is reversed so then this logic is as well
    SmartDashboard.putNumber("MoveAlgaeArmSpeed", speed);
    algae1.set(ControlMode.PercentOutput, speed);
    
/*     if ((encoderMax > algaeEncoder.getDistance()) && speed < 0)
    {
      System.out.println("Go this way");
      System.out.println(thiswaycount++);
      algae1.set(ControlMode.PercentOutput, speed);
    }
    else if ((algaeEncoder.getDistance() > encoderMin) && speed > 0)
    {
      System.out.printf("Go that way");
      System.out.println(thatwaycount++);
      algae1.set(ControlMode.PercentOutput, speed);
    } */
  }

public void ingestAlgae(double speed) {
  /* lower the algae which will set the motor in the proper direction to lower
   * this may need to take in a speed and the PID logic for a lower to x level with
   * the PID slowing the speed on approach
   */
  algaeWheels.set(ControlMode.PercentOutput, speed);

} 

public void regurgitateAlgae(double speed) {
  /* lower the algae which will set the motor in the proper direction to lower
   * this may need to take in a speed and the PID logic for a lower to x level with
   * the PID slowing the speed on approach
   */
  algaeWheels.set(ControlMode.PercentOutput, -speed);

}

/* public double getAlgaeEncoder() {
  // lower the algae which will set the motor in the proper direction to lower
   // this may need to take in a speed and the PID logic for a lower to x level with
   // the PID slowing the speed on approach
   //
  return algaeEncoder.getDistance();

}

public void resetAlgaeEncoder() {
  // lower the algae which will set the motor in the proper direction to lower
   // this may need to take in a speed and the PID logic for a lower to x level with
   // the PID slowing the speed on approach
   //
  algaeEncoder.reset();

} */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("periodic");

    //SmartDashboard.putNumber("Algaeencoderdistance", algaeEncoder.getDistance());
   
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
