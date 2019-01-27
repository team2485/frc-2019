/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class CargoEject extends Command {
  
  long ejectTime;

  double ejectPWM; //negative

  long start;

  public CargoEject() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(RobotMap.cargoIntake);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotMap.cargoIntake.setIntakeManual(ejectPWM);

    start = System.currentTimeMillis();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // return false;

    if (System.currentTimeMillis() - start >= ejectTime) {
      return true;
    }

    return false;
    
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    RobotMap.cargoIntake.setIntakeManual(0);
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
