// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class R2Jesu_AlgaeSubsystem extends SubsystemBase {
  /** Creates a new R2Jesu_ElevatorSubsystem. */

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
  /* 
}  
#include "Robot.h"

int loopy;

void Robot::R2Jesu_Arm()
{
  frc::SmartDashboard::PutNumber("fullSpeed", fullSpeed);
  if (fullSpeed <= .2 || frc::DriverStation::IsAutonomousEnabled()) 
  //if the full speed is less than or equal to .2, then FRC and drivers station is Autonomous 
  {
     if (gridPad.GetRawButton(8)) {
         if (armX > 1) 
         {
           m_armController.SetPID(upPpid, upIpid, upDpid);
           //if you press button 8 and the arm is greater than 1 then the arm goes up 
         }
         else if (armX < 1)
         {
           m_armController.SetPID(downPpid, downIpid, downDpid);
           //if the arm is less than 1 the arm goes down 
         }
   
         armX=1;
         armSetPoint=armStops[armX];
         //if the arm is exactly 1 then the arm stops 
     }
     if (gridPad.GetRawButton(7) || gridPad.GetRawButton(9)) {
       if (gridPad.GetRawButton(9)) {
         if (armSetPoint < 0.91) {
            armSetPoint=armSetPoint + 0.0003;
            m_armController.Reset();
            m_armController.SetPID(upPpid, upIpid, upDpid);
            //if you press buttons 7 and 9 and the arm is less than 0.91 then the arm will go up .0003
       }
       else {
         if (armSetPoint > .1) {
            armSetPoint=armSetPoint - 0.0003;
            m_armController.Reset();
            m_armController.SetPID(downPpid, downIpid, downDpid);
            // if the arm is greater than .1 then the arm goes down.0003
         }
       }
       for (loopy=0;loopy < (sizeof(armStops) / sizeof(double)); loopy++)
       {
         if (armSetPoint > armStops[loopy])
         {
           armX=loopy;
           break;
           // if the arm set point is zero the arm stops
         }
       }
     }
     else {
       if (m_Operatorstick.GetTriangleButtonPressed()) {
         if (armX > 0) {
           armX--;
           armSetPoint=armStops[armX];
           m_armController.SetPID(upPpid, upIpid, upDpid);
           // if triangle button is pressed amd arm is greater than 0, then the arm stops 
           // and then you can make the arm go up
         }
       }
       if (m_Operatorstick.GetCrossButtonPressed()) {
         if (armX < ((sizeof(armStops) / sizeof(double)) - 1)) {
             armX++;
             armSetPoint=armStops[armX];
             m_armController.SetPID(downPpid, downIpid, downDpid);
           }
           //if the cross button is pressed and the arm is less then 1 then the arm stops
           //and then you can make the arm go down 
       }
       if (gridPad.GetRawButtonPressed(1) || gridPad.GetRawButtonPressed(2) || gridPad.GetRawButtonPressed(3)) {
         if (armX > 3) 
         {
           m_armController.SetPID(upPpid, upIpid, upDpid);
         }
         //if you press the buttons 1, 2, and 3 and the arm is greater than 3 then the arm goes up 
         else if (armX < 3)
         {
           m_armController.SetPID(downPpid, downIpid, downDpid);
         }
           //if you press the buttons 1, 2, and 3 and the arm is less than 3 then the arm goes down 
   
         armX=3;
         armSetPoint=armStops[armX];
       }
       //if the arm equals 3 then the arm stops 
       if (gridPad.GetRawButtonPressed(4) || gridPad.GetRawButtonPressed(5) || gridPad.GetRawButtonPressed(6)) {
         if (armX > 2) 
         {
           m_armController.SetPID(upPpid, upIpid, upDpid);
         }
         //if you press the buttons 4, 5, and 6 and the arm is greater than 2 then the arm goes up 
         else if (armX < 2)
         {
           m_armController.SetPID(downPpid, downIpid, downDpid);
         }
         //if you press the buttons 4, 5, and 6, and the arm is less than 2 the arm goes down
         armX=2;
         armSetPoint=armStops[armX];
       }
   
     }
     armPidOutput = m_armController.Calculate((m_encArm.GetAbsolutePosition()), armSetPoint);
     if (((armX == 0) || (armSetPoint == 0.91)) && !(frc::DriverStation::IsAutonomousEnabled()))
     {
       armPidOutput = armPidOutput * .5;
     }
     if (armX == 1)
     {
       armPidOutput = armPidOutput * .5;
     }
   
     armMotor.Set(armPidOutput);
     // if arm = 0 or arm = .91 and it is not autonomous then it increases .5
     // arm = 1 then it increases .5
  }
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
