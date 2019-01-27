/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2485.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.usfirst.frc.team2485.util.TalonSRXWrapper;
import org.usfirst.frc.team2485.util.SpeedControllerWrapper;
import org.usfirst.frc.team2485.robot.subsystems.CargoIntake;
import org.usfirst.frc.team2485.robot.subsystems.DriveTrain;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import org.usfirst.frc.team2485.util.PigeonWrapperRateAndAngle;
import org.usfirst.frc.team2485.util.EncoderWrapperRateAndDistance;
import  org.usfirst.frc.team2485.util.LidarWrapper;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.I2C.Port;
import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.Solenoid;


import org.usfirst.frc.team2485.util.PigeonWrapperRateAndAngle.Units;
import  org.usfirst.frc.team2485.robot.subsystems.DriveTrain;
import  org.usfirst.frc.team2485.robot.subsystems.Intake;
import  org.usfirst.frc.team2485.robot.subsystems.HatchPanelArm;
import org.usfirst.frc.team2485.robot.subsystems.HatchPanelIntake;
import org.usfirst.frc.team2485.util.TalonSRXEncoderWrapper;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */


public class RobotMap {
	public static final int WHEEL_RADIUS = 3;

	public static TalonSRX driveLeftTalon1; 
	public static TalonSRX driveLeftTalon2; 
	public static TalonSRX driveLeftTalon3; 
	public static TalonSRX driveLeftTalon4;

	public static TalonSRX driveRightTalon1;
	public static TalonSRX driveRightTalon2;
	public static TalonSRX driveRightTalon3;
	public static TalonSRX driveRightTalon4;


	public static TalonSRXWrapper driveLeftTalonWrapperPWM1;
	public static TalonSRXWrapper driveLeftTalonWrapperPWM2;
	public static TalonSRXWrapper driveLeftTalonWrapperPWM3;
	public static TalonSRXWrapper driveLeftTalonWrapperPWM4;

	public static TalonSRXWrapper driveRightTalonWrapperPWM1;
	public static TalonSRXWrapper driveRightTalonWrapperPWM2;
	public static TalonSRXWrapper driveRightTalonWrapperPWM3;
	public static TalonSRXWrapper driveRightTalonWrapperPWM4;

	public static TalonSRXWrapper driveRightTalonWrapperCurrent1;
	public static TalonSRXWrapper driveRightTalonWrapperCurrent2;
	public static TalonSRXWrapper driveRightTalonWrapperCurrent3;
	public static TalonSRXWrapper driveRightTalonWrapperCurrent4;

	public static TalonSRXWrapper driveLeftTalonWrapperCurrent1;
	public static TalonSRXWrapper driveLeftTalonWrapperCurrent2;
	public static TalonSRXWrapper driveLeftTalonWrapperCurrent3;
	public static TalonSRXWrapper driveLeftTalonWrapperCurrent4;

	public static TalonSRXWrapper[] driveRightTalonWrappersCurrent;
	public static TalonSRXWrapper[] driveLeftTalonWrappersCurrent;

	public static TalonSRXWrapper[] driveRightTalonWrappersPWM;
	public static TalonSRXWrapper[] driveLeftTalonWrappersPWM;

	public static SpeedControllerWrapper driveLeftCurrent;
	public static SpeedControllerWrapper driveRightCurrent;

	public static SpeedControllerWrapper driveLeftPWM;
	public static SpeedControllerWrapper driveRightPWM;
	
	//Color Sensor
	public static byte[] colorSensorOutput;

	//Intake
	public static TalonSRX leftIntakeTalon;
	public static TalonSRX rightIntakeTalon;

	public static TalonSRXWrapper leftIntakeTalonWrapper;
	public static TalonSRXWrapper rightIntakeTalonWrapper;

	public static TalonSRXWrapper[] leftIntakeTalonWrappers;
	public static TalonSRXWrapper[] rightIntakeTalonWrappers;

	public static Encoder driveLeftEncoder;
	public static Encoder driveRightEncoder;
	public static EncoderWrapperRateAndDistance driveLeftEncoderWrapperRate;
	public static EncoderWrapperRateAndDistance driveRightEncoderWrapperRate;
	public static EncoderWrapperRateAndDistance driveLeftEncoderWrapperDistance;
	public static EncoderWrapperRateAndDistance driveRightEncoderWrapperDistance;

	public static SpeedControllerWrapper intakeLeft;
	public static SpeedControllerWrapper intakeRight;

	
	//Hatch Panel Arm

	public static TalonSRX hatchPanelArmTalon;
	public static TalonSRXWrapper hatchPanelArmWrapper;
	public static TalonSRXEncoderWrapper hatchPanelArmEncoderWrapperDistance;
	public static TalonSRXEncoderWrapper hatchPanelArmEncoderWrapperRate;
	public static SpeedControllerWrapper hatchPanelArmSC;


	public static PigeonIMU gyro;
	public static PigeonWrapperRateAndAngle gyroRateWrapper;
	public static PigeonWrapperRateAndAngle gyroAngleWrapper;
	
	public static LidarWrapper lidar;

	public static I2C colorSensor; 


	public static Solenoid centerSolenoidIn; 
	public static Solenoid centerSolenoidOut; 
	public static Solenoid sideSolenoidIn;
	public static Solenoid sideSolenoidOut;
	public static Solenoid suctionSolenoid;

	//Subsystems
	public static DriveTrain driveTrain;
	public static Intake intake;
	public static CargoIntake cargoIntake;
	public static HatchPanelIntake hatchPanelIntake;
	public static HatchPanelArm hatchPanelArm;

	public static TalonSRX cargoIntakeTalon;
	public static TalonSRXWrapper cargoIntakeTalonWrapper; 

	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	public static void init(){
		driveLeftTalon1 = new TalonSRX(11);
		driveLeftTalon2 = new TalonSRX(12);
		driveLeftTalon3 = new TalonSRX(13);
		driveLeftTalon4 = new TalonSRX(14);

		driveRightTalon1 = new TalonSRX(4);
		driveRightTalon2 = new TalonSRX(5);
		driveRightTalon3 = new TalonSRX(6);
		driveRightTalon4 = new TalonSRX(7);

		driveLeftTalonWrappersCurrent = new TalonSRXWrapper[4];
		driveRightTalonWrappersCurrent = new TalonSRXWrapper[4];

		driveRightTalonWrappersPWM = new TalonSRXWrapper[4];
		driveLeftTalonWrappersPWM = new TalonSRXWrapper[4];


		driveLeftTalonWrapperPWM1 = new TalonSRXWrapper(ControlMode.PercentOutput,driveLeftTalon1);
		driveLeftTalonWrapperPWM2 = new TalonSRXWrapper(ControlMode.PercentOutput,driveLeftTalon2);
		driveLeftTalonWrapperPWM3 = new TalonSRXWrapper(ControlMode.PercentOutput,driveLeftTalon3);
		driveLeftTalonWrapperPWM4 = new TalonSRXWrapper(ControlMode.PercentOutput,driveLeftTalon4);
		driveRightTalonWrapperPWM1 = new TalonSRXWrapper(ControlMode.PercentOutput,driveRightTalon1);
		driveRightTalonWrapperPWM2 = new TalonSRXWrapper(ControlMode.PercentOutput,driveRightTalon2);
		driveRightTalonWrapperPWM3 = new TalonSRXWrapper(ControlMode.PercentOutput,driveRightTalon3);
		driveRightTalonWrapperPWM4 = new TalonSRXWrapper(ControlMode.PercentOutput,driveRightTalon4);

		driveLeftTalonWrapperCurrent1 = new TalonSRXWrapper(ControlMode.Current,driveLeftTalon1);
		driveLeftTalonWrapperCurrent2 = new TalonSRXWrapper(ControlMode.Current,driveLeftTalon2);
		driveLeftTalonWrapperCurrent3 = new TalonSRXWrapper(ControlMode.Current,driveLeftTalon3);
		driveLeftTalonWrapperCurrent4 = new TalonSRXWrapper(ControlMode.Current,driveLeftTalon4);
		driveRightTalonWrapperCurrent1 = new TalonSRXWrapper(ControlMode.Current,driveRightTalon1);
		driveRightTalonWrapperCurrent2 = new TalonSRXWrapper(ControlMode.Current,driveRightTalon2);
		driveRightTalonWrapperCurrent3 = new TalonSRXWrapper(ControlMode.Current,driveRightTalon3);
		driveRightTalonWrapperCurrent4 = new TalonSRXWrapper(ControlMode.Current,driveRightTalon4);

		






		
		driveLeftTalonWrappersCurrent[0]= driveLeftTalonWrapperCurrent1;
		driveLeftTalonWrappersCurrent[1]= driveLeftTalonWrapperCurrent2;
		driveLeftTalonWrappersCurrent[2]= driveLeftTalonWrapperCurrent3;
		driveLeftTalonWrappersCurrent[3]= driveLeftTalonWrapperCurrent4;
		
		driveRightTalonWrappersCurrent[0]= driveRightTalonWrapperCurrent1;
		driveRightTalonWrappersCurrent[1]= driveRightTalonWrapperCurrent2;
		driveRightTalonWrappersCurrent[2]= driveRightTalonWrapperCurrent3;
		driveRightTalonWrappersCurrent[3]= driveRightTalonWrapperCurrent4;


		driveLeftTalonWrappersPWM[0] = driveLeftTalonWrapperPWM1;
		driveLeftTalonWrappersPWM[1] = driveLeftTalonWrapperPWM2;
		driveLeftTalonWrappersPWM[2] = driveLeftTalonWrapperPWM3;
		driveLeftTalonWrappersPWM[3] = driveLeftTalonWrapperPWM4;

		driveRightTalonWrappersPWM[0] = driveRightTalonWrapperPWM1;
		driveRightTalonWrappersPWM[1] = driveRightTalonWrapperPWM2;
		driveRightTalonWrappersPWM[2] = driveRightTalonWrapperPWM3;
		driveRightTalonWrappersPWM[3] = driveRightTalonWrapperPWM4;



	

		
		driveLeftCurrent = new SpeedControllerWrapper(driveLeftTalonWrappersCurrent);
		driveRightCurrent = new SpeedControllerWrapper(driveRightTalonWrappersCurrent);	
		driveLeftPWM = new SpeedControllerWrapper(driveLeftTalonWrappersPWM);
		driveRightPWM = new SpeedControllerWrapper(driveRightTalonWrappersPWM);

		driveLeftCurrent.setInverted(true);
		driveLeftTalonWrapperCurrent3.setInverted(false);


		driveLeftPWM.setInverted(true);
		// driveLeftTalonWrapperPWM3.setInverted(false);

		leftIntakeTalon = new TalonSRX(1);
		rightIntakeTalon = new TalonSRX(2);


		

		gyro = new PigeonIMU(0); //port known because Ian did something

		gyroRateWrapper = new PigeonWrapperRateAndAngle(gyro, PIDSourceType.kRate, Units.RADS);
		gyroAngleWrapper = new PigeonWrapperRateAndAngle(gyro, PIDSourceType.kDisplacement, Units.RADS);

		driveLeftEncoder = new Encoder(0, 1);
		driveRightEncoder = new Encoder(2, 3);

		driveLeftEncoder.setDistancePerPulse(WHEEL_RADIUS*2*Math.PI/250);
		driveRightEncoder.setDistancePerPulse(WHEEL_RADIUS*2*Math.PI/250);

		driveLeftEncoder.setReverseDirection(true);
		driveRightEncoder.setReverseDirection(false);

		driveLeftEncoderWrapperDistance = new EncoderWrapperRateAndDistance(driveLeftEncoder, PIDSourceType.kDisplacement);
		driveLeftEncoderWrapperRate = new EncoderWrapperRateAndDistance(driveLeftEncoder, PIDSourceType.kRate);
		driveRightEncoderWrapperDistance = new EncoderWrapperRateAndDistance(driveRightEncoder, PIDSourceType.kDisplacement);
		driveRightEncoderWrapperRate = new EncoderWrapperRateAndDistance(driveRightEncoder, PIDSourceType.kRate);

		

		// driveLeftEncoderWrapperDistance.setGearRatio(16/62);
		// driveLeftEncoderWrapperRate.setGearRatio(16/62);
		// driveRightEncoderWrapperDistance.setGearRatio(16/62);
		// driveRightEncoderWrapperRate.setGearRatio(16/62);

		colorSensor = new I2C(I2C.Port.kOnboard, 0x3C);

		driveTrain = new DriveTrain();

		//Intake Motors
		leftIntakeTalon = new TalonSRX(1);
		rightIntakeTalon = new TalonSRX(2);

		leftIntakeTalonWrapper = new TalonSRXWrapper(ControlMode.PercentOutput,leftIntakeTalon);
		rightIntakeTalonWrapper = new TalonSRXWrapper(ControlMode.PercentOutput,rightIntakeTalon);
		
		leftIntakeTalonWrappers = new TalonSRXWrapper[1];
		rightIntakeTalonWrappers = new TalonSRXWrapper[1];
		leftIntakeTalonWrappers[0]= leftIntakeTalonWrapper; //Unknown port
		rightIntakeTalonWrappers[0]= rightIntakeTalonWrapper; //Unknown port

		intakeRight = new SpeedControllerWrapper(rightIntakeTalonWrappers);
		intakeLeft = new SpeedControllerWrapper(leftIntakeTalonWrappers);
		

		lidar = new LidarWrapper(I2C.Port.kOnboard); //port unknown

		centerSolenoidIn = new Solenoid(1);
		centerSolenoidOut = new Solenoid(2);
		sideSolenoidIn = new Solenoid(3);
		sideSolenoidOut = new Solenoid(4);
		suctionSolenoid = new Solenoid(5);

		hatchPanelIntake = new HatchPanelIntake(); 
		hatchPanelArm = new HatchPanelArm();

		hatchPanelArmTalon = new TalonSRX(9);
		hatchPanelArmWrapper = new TalonSRXWrapper(ControlMode.PercentOutput, hatchPanelArmTalon);
		hatchPanelArmEncoderWrapperDistance = new TalonSRXEncoderWrapper(hatchPanelArmTalon, PIDSourceType.kDisplacement);
		hatchPanelArmEncoderWrapperRate = new TalonSRXEncoderWrapper(hatchPanelArmTalon, PIDSourceType.kRate);
		hatchPanelArmSC = new SpeedControllerWrapper(hatchPanelArmWrapper);

		cargoIntakeTalon = new TalonSRX(8);
		cargoIntakeTalonWrapper = new TalonSRXWrapper(ControlMode.PercentOutput, cargoIntakeTalon);

		cargoIntake = new CargoIntake();
		intake = new Intake();
	}

	// public static void updateConstants(){
	// 	RobotMap.driveTrain.updateConstants();
	// }
}
