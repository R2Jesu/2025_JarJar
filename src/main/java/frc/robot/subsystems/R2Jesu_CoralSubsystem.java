// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;



public class R2Jesu_CoralSubsystem extends SubsystemBase {
  private TalonSRX coralRight = new TalonSRX(13);
  private TalonSRX coralLeft = new TalonSRX(14);
  private WPI_VictorSPX coralChute = new WPI_VictorSPX(16);
  private DigitalInput backSensor = new DigitalInput(11);
  private DigitalInput frontSensor = new DigitalInput(13);
  private Boolean overrideSensor=false;
  private Boolean haveCoral=false;
  
  
  /** Creates a new R2Jesu_CoralSubsystem. */

  /** Here we will eventuall put the motor defintions that we need to control to raise and lower the coral */
 


  /**
   * R2Jesu_Coral command factory method.
   *
   * @return a command
   */
  public Command R2Jesu_CoralMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An R2Jesu_Coral method querying a boolean state of the subsystem (for R2Jesu_Coral, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean R2Jesu_CoralCondition() {
    // Query some boolean state, such as a digital sensor.
    return haveCoral;
  }

  public void releaseCoral() {
    overrideSensor=true;
  }

  public void dropChute() {
    coralChute.set(ControlMode.PercentOutput, -1.0);
  }

  public void stopChute() {
    coralChute.set(ControlMode.PercentOutput, 0.0);
  }

  public Boolean hasReleased() {
    if (overrideSensor == false) {
      return true;
    }
    else {
      return false;
    }
  }

  public Boolean haveCoral() {
    return this.haveCoral;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //System.out.println("periodic");

    if (backSensor.get() && !frontSensor.get() && !overrideSensor) {
      coralLeft.set(ControlMode.PercentOutput, 0.0);
      coralRight.set(ControlMode.PercentOutput, -0.0);
      haveCoral=true;
    } else {
      if (overrideSensor) {
        if (R2Jesu_ElevatorSubsystem.getElevatorLevel() == 3) {
          coralLeft.set(ControlMode.PercentOutput, 0.5);
          coralRight.set(ControlMode.PercentOutput, -0.5);
        } else {
          coralLeft.set(ControlMode.PercentOutput, 0.8);
          coralRight.set(ControlMode.PercentOutput, -0.8);
        }
      } else {
        coralLeft.set(ControlMode.PercentOutput, 0.5);
        coralRight.set(ControlMode.PercentOutput, -0.5);       
      }
      haveCoral=false;
    }

    if (backSensor.get() && frontSensor.get() && !(R2Jesu_ElevatorSubsystem.getElevatorLevel() == 0)) {
      coralLeft.set(ControlMode.PercentOutput, 0.0);
      coralRight.set(ControlMode.PercentOutput, -0.0);
    }  

    if (frontSensor.get()) {
      overrideSensor=false;
    }
  
    SmartDashboard.putBoolean("BackSensor", backSensor.get());
    SmartDashboard.putBoolean("FrontSensor", frontSensor.get());
    SmartDashboard.putBoolean("Override", overrideSensor);
    SmartDashboard.putBoolean("haveCoral", haveCoral);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
