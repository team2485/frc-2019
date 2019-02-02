/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import edu.wpi.first.wpilibj.command.Subsystem;



/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public TransferNode elevatorTN; //what do we name this? 
  public TransferNode distanceTN;
  
  public WarlordsPIDController elevatorDistPID;  
  public WarlordsPIDController elevatorVelocityPID;

  public PIDSourceWrapper distancePIDSource;
  public PIDSourceWrapper velocityPIDSource; 

  
   
  double elevatorPos1; //lower hatch
  double elevatorPos2; //middle hatch
  double elevatorPos3; //upper hatch
  double elevatorPos4; //lower cargo
  double elevatorPos5; //middle cargo
  double elevatorPos6; //upper cargo

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public static void movePWM() {
        RobotMap.elevatorTalonWrapperPWM1.set(0.5);
        RobotMap.elevatorTalonWrapperPWM2.set(0.5);
    }


}
