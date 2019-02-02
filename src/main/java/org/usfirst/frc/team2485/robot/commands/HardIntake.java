/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2485.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import  org.usfirst.frc.team2485.robot.subsystems.Intake;
import  org.usfirst.frc.team2485.robot.RobotMap;

/**
 * Add your docs here.
 */
public class HardIntake extends InstantCommand {
  /**
   * Add your docs here.
   */
  public HardIntake() {
    super();
    requires(RobotMap.intake);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    RobotMap.intake.hardIntake();
  }

}
