/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2485.robot.commands;

import edu.wpi.first.wpilibj.command.Command;

public class CargoIntake extends Command {
  //constants
  long intakeTime; 
  long minTimeSpike; //in milliseconds
  double intakePWM; //positive
  long startTimeSpike;
  double minSpikeRatio = 1.05;
  double minSpikeDecrease = 1.02;
  
  //non-constants
  long start;
  double lastCurrent;
  double currentCurrent;
  boolean firstSpikeCompleted;
  boolean isSpiking;
  boolean spike = false;
  public CargoIntake() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);

    requires(RobotMap.cargoIntake);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    RobotMap.cargoIntake.setIntakeManual(intakePWM);
    start = System.currentTimeMillis();

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override 
  protected boolean isFinished() {
    currentCurrent = cargoIntake.current();

    if(currentCurrent >= minSpikeRatio * lastCurrent) {
      startTimeSpike = System.currentTimeMillis();
      isSpiking = true;
    }

    if(isSpiking) {
      if (cargoIntake.current() < currentCurrent / minSpikeDecrease) {
        isSpiking = false;
      }

      if (System.currentTimeMillis() - startSpike >= minTimeSpike) {
        spike = true;
        isSpiking = false;
      }
    }


    
    if(spike && firstSpikeCompleted){
      return true;
    }
    else if(spike){
      firstSpikeCompleted = true;
      return false;
    }

    if (system.currentTimeMillis() - start > intakeTime) {
      return true;
    }

    lastCurrent = currentCurrent;
    spike = false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    RobotMap.cargoIntake.setIntakeManual(0);
  }

  // protected boolean spike (){
  //   currentCurrent = cargoIntake.current();

  //   if(currentCurrent >= minSpikeRatio * lastCurrent) {
  //     long startSpike = System.currentTimeMillis();
  //     while (!(System.currentTimeMillis() - startSpike >= minTimeSpike)) {
  //       if (cargoIntake.current() < currentCurrent / minSpikeDecrease) {
  //         return false;
  //       }
  //     }
      
  //     return true;

  //   }

  // }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
