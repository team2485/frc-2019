<<<<<<< HEAD
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2485.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import  org.usfirst.frc.team2485.util.TalonSRXWrapper;
import  org.usfirst.frc.team2485.util.SpeedControllerWrapper;
import org.usfirst.frc.team2485.robot.subsystems.Drivetrain;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */


public class RobotMap {
	public static TalonSRX driveLeftTalon1; 
	public static TalonSRX driveLeftTalon2; 
	public static TalonSRX driveLeftTalon3; 

	public static TalonSRX driveRightTalon1;
	public static TalonSRX driveRightTalon2;
	public static TalonSRX driveRightTalon3;

	public static TalonSRXWrapper driveLeftTalonWrapper1;
	public static TalonSRXWrapper driveLeftTalonWrapper2;
	public static TalonSRXWrapper driveLeftTalonWrapper3;

	public static TalonSRXWrapper driveRightTalonWrapper1;
	public static TalonSRXWrapper driveRightTalonWrapper2;
	public static TalonSRXWrapper driveRightTalonWrapper3;

	public static TalonSRXWrapper[] driveRightTalonWrappers;
	public static TalonSRXWrapper[] driveLeftTalonWrappers;
	
	public static SpeedControllerWrapper driveLeft;
	public static SpeedControllerWrapper driveRight;
	
	public static Drivetrain drive;

	public static PigeonIMU gyro;
	public static PigeonWrapperRateAndAngle gyroWrapper;
	
	public static LidarWrapper lidar;

	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	public void init(){
		driveLeftTalon1 = new TalonSRX(1);
		driveLeftTalon2 = new TalonSRX(2);
		driveLeftTalon3 = new TalonSRX(3);

		driveRightTalon1 = new TalonSRX(4);
		driveRightTalon2 = new TalonSRX(5);
		driveRightTalon3 = new TalonSRX(6);

		driveLeftTalonWrapper1 = new TalonSRXWrapper(ControlMode.PercentOutput,driveLeftTalon1);
		driveLeftTalonWrapper2 = new TalonSRXWrapper(ControlMode.PercentOutput,driveLeftTalon2);
		driveLeftTalonWrapper3 = new TalonSRXWrapper(ControlMode.PercentOutput,driveLeftTalon3);
		driveRightTalonWrapper1 = new TalonSRXWrapper(ControlMode.PercentOutput,driveRightTalon1);
		driveRightTalonWrapper2 = new TalonSRXWrapper(ControlMode.PercentOutput,driveRightTalon2);
		driveRightTalonWrapper3 = new TalonSRXWrapper(ControlMode.PercentOutput,driveRightTalon3);

		driveLeftTalonWrappers[0]= driveLeftTalonWrapper1;
		driveLeftTalonWrappers[1]= driveLeftTalonWrapper2;
		driveLeftTalonWrappers[2]= driveLeftTalonWrapper3;
		
		driveRightTalonWrappers[0]= driveRightTalonWrapper1;
		driveRightTalonWrappers[1]= driveRightTalonWrapper2;
		driveRightTalonWrappers[2]= driveRightTalonWrapper3;
		
		driveLeft = new SpeedControllerWrapper(driveLeftTalonWrappers);
		driveRight = new SpeedControllerWrapper(driveRightTalonWrappers);				
		
		drive = new Drivetrain();

		gyro = new PigeonIMU(0); //port unknown

		gyroWrapper = new PigeonWrapperRateAndAngle(gyro, PIDSourceType.kRate, Units.RADS);

		lidar = new LidarWrapper(8); //port unknown
	}
}
=======
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2485.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import  org.usfirst.frc.team2485.util.TalonSRXWrapper;
import org.usfirst.frc.team2485.util.AutoPath;
import  org.usfirst.frc.team2485.util.SpeedControllerWrapper;

import org.usfirst.frc.team2485.robot.subsystems.Drivetrain;
import com.qualcomm.robotcore.hardware.ColorSensor;


/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */


public class RobotMap {
	public static TalonSRX driveLeftTalon1; 
	public static TalonSRX driveLeftTalon2; 
	public static TalonSRX driveLeftTalon3; 
	public static TalonSRX driveRightTalon1;
	public static TalonSRX driveRightTalon2;
	public static TalonSRX driveRightTalon3;
	public static TalonSRXWrapper driveLeftTalonWrapper1;
	public static TalonSRXWrapper driveLeftTalonWrapper2;
	public static TalonSRXWrapper driveLeftTalonWrapper3;
	public static TalonSRXWrapper driveRightTalonWrapper1;
	public static TalonSRXWrapper driveRightTalonWrapper2;
	public static TalonSRXWrapper driveRightTalonWrapper3;
	public static TalonSRXWrapper[] driveRightTalonWrappers;
	public static TalonSRXWrapper[] driveLeftTalonWrappers;
	
	public static SpeedControllerWrapper driveLeft;
	public static SpeedControllerWrapper driveRight;
	


	//Sensors
	public static ColorSensor colorSensor; 



	public static DriveTrain driveTrain;
	

	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	public void init(){
		driveLeftTalon1 = new TalonSRX(1);
		driveLeftTalon2 = new TalonSRX(2);
		driveLeftTalon3 = new TalonSRX(3);
		driveRightTalon1 = new TalonSRX(4);
		driveRightTalon2 = new TalonSRX(5);
		driveRightTalon3 = new TalonSRX(6);
		driveLeftTalonWrapper1 = new TalonSRXWrapper(ControlMode.PercentOutput,driveLeftTalon1);
		driveLeftTalonWrapper2 = new TalonSRXWrapper(ControlMode.PercentOutput,driveLeftTalon2);
		driveLeftTalonWrapper3 = new TalonSRXWrapper(ControlMode.PercentOutput,driveLeftTalon3);
		driveRightTalonWrapper1 = new TalonSRXWrapper(ControlMode.PercentOutput,driveRightTalon1);
		driveRightTalonWrapper2 = new TalonSRXWrapper(ControlMode.PercentOutput,driveRightTalon2);
		driveRightTalonWrapper3 = new TalonSRXWrapper(ControlMode.PercentOutput,driveRightTalon3);

		driveLeftTalonWrappers[0]= driveLeftTalonWrapper1;
		driveLeftTalonWrappers[1]= driveLeftTalonWrapper2;
		driveLeftTalonWrappers[2]= driveLeftTalonWrapper3;
		
		driveRightTalonWrappers[0]= driveRightTalonWrapper1;
		driveRightTalonWrappers[1]= driveRightTalonWrapper2;
		driveRightTalonWrappers[2]= driveRightTalonWrapper3;
		

		
		driveLeft = new SpeedControllerWrapper(driveLeftTalonWrappers);
		driveRight = new SpeedControllerWrapper(driveRightTalonWrappers);
		
		//Sensors
		colorSensor = new ColorSensor();
		colorSensor.setI2cAddress(/*some number*/);
		
	
		 
		
		driveTrain = new DriveTrain();

	}
}
>>>>>>> f692981e01f66865bd909c104692182d407aed7a
