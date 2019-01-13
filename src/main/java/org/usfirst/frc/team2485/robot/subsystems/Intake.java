/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static void hardIntake() {
		double doubleintake = 0.7;
		RobotMap.intakeLeft.set(doubleintake);
		RobotMap.intakeRight.set(-doubleintake);
	}

	public static void hardEject() {

		double doubleeject = -0.7;
		RobotMap.intakeLeft.set(doubleeject);
		RobotMap.intakeRight.set(-doubleeject);
	}

  public static void softIntake() {
		double doubleintake = 0.3;
		RobotMap.intakeLeft.set(doubleintake);
		RobotMap.intakeRight.set(-doubleintake);
	}

	public static void softEject() {

		double doubleeject = -0.3;
		RobotMap.intakeLeft.set(doubleeject);
		RobotMap.intakeRight.set(-doubleeject);
  }
  
  public static void stopIntake() {
		double doubleintake = 0;
		RobotMap.intakeLeft.set(doubleintake);
		RobotMap.intakeRight.set(-doubleintake);
	}

	public static void setIntake(double d) {

		double doubleeject = d;
		RobotMap.intakeLeft.set(doubleeject);
		RobotMap.intakeRight.set(-doubleeject);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
