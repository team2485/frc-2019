/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2485.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import  org.usfirst.frc.team2485.robot.subsystems.Intake;
import org.usfirst.frc.team2485.util.ThresholdHandler;
import org.usfirst.frc.team2485.robot.OI;
import  org.usfirst.frc.team2485.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ElevatorMove extends Command {
  /**
   * Add your docs here.
   */
  public ElevatorMove() {
    super();
    requires(RobotMap.elevator);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    setInterruptible(true);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    
  }

  public void execute() {
    double power = ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(2)-OI.suraj.getRawAxis(3), .1, 0, 0.5);
    System.out.println("Power: " + power);
    RobotMap.elevator.setPWM(power);
    
  }

  public boolean isFinished() {
    return false;
  }

}
