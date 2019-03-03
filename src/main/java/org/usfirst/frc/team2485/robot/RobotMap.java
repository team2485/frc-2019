/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2485.robot;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import org.usfirst.frc.team2485.robot.subsystems.CargoArm;
import org.usfirst.frc.team2485.robot.subsystems.CargoRollers;
import org.usfirst.frc.team2485.robot.subsystems.DriveTrain;
import org.usfirst.frc.team2485.robot.subsystems.Elevator;
import org.usfirst.frc.team2485.robot.subsystems.HatchIntake;
import org.usfirst.frc.team2485.util.EncoderWrapperRateAndDistance;
import org.usfirst.frc.team2485.util.PigeonWrapperRateAndAngle;
import org.usfirst.frc.team2485.util.SpeedControllerWrapper;
import org.usfirst.frc.team2485.util.TalonSRXWrapper;
import org.usfirst.frc.team2485.util.PigeonWrapperRateAndAngle.Units;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;




/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */


public class RobotMap {
	//ROBOT CONSTANTS
	public static int DRUM_RADIUS = 1;

	//PORTS
	//Speed Controllers
	public static int driveLeftTalonPort1 = 11;
	public static int driveLeftTalonPort2 = 12;
	public static int driveLeftTalonPort3 = 13;
	public static int driveLeftTalonPort4 = 14;

	public static int driveRightTalonPort1 = 4;
	public static int driveRightTalonPort2 = 5;
	public static int driveRightTalonPort3 = 6;
	public static int driveRightTalonPort4 = 7;

	public static int elevatorTalonPort1 = 2;
	public static int elevatorTalonPort2 = 3;

	public static int gyroTalonPort = 1;

	public static int cargoArmTalonPort = 9;

	public static int cargoRollersTalonPort = 8;

	public static int liftSolenoidPortIn = 0, liftSolenoidPortOut = 4;
	public static int slideSolenoidPortIn = 1, slideSolenoidPortOut = 5;
	public static int pushersSolenoidPortIn = 2, pushersSolenoidPortOut = 6;
	public static int hookSolenoidPortIn = 3, hookSolenoidPortOut = 7;

	//Sensors
	public static int driveLeftEncoderPort1 = 0, driveLeftEncoderPort2 = 1;
	public static int driveRightEncoderPort1 = 2, driveRightEncoderPort2 = 3;

	public static int elevatorEncoderPort1 = 6, elevatorEncoderPort2 = 7;

	public static int cargoArmEncoderPort1 = 4, cargoArmEncoderPort2 = 5;



	//DRIVE TRAIN
	public static TalonSRX driveLeftTalon1;
	public static TalonSRX driveLeftTalon2;
	public static TalonSRX driveLeftTalon3;
	public static TalonSRX driveLeftTalon4;

	public static TalonSRX driveRightTalon1;
	public static TalonSRX driveRightTalon2;
	public static TalonSRX driveRightTalon3;
	public static TalonSRX driveRightTalon4;

	public static TalonSRX gyroTalon;

	public static TalonSRXWrapper driveLeftTalonWrapperCurrent1;
	public static TalonSRXWrapper driveLeftTalonWrapperCurrent2;
	public static TalonSRXWrapper driveLeftTalonWrapperCurrent3;
	public static TalonSRXWrapper driveLeftTalonWrapperCurrent4;

	public static TalonSRXWrapper driveRightTalonWrapperCurrent1;
	public static TalonSRXWrapper driveRightTalonWrapperCurrent2;
	public static TalonSRXWrapper driveRightTalonWrapperCurrent3;
	public static TalonSRXWrapper driveRightTalonWrapperCurrent4;

	public static TalonSRXWrapper driveLeftTalonWrapperPercentOutput1;
	public static TalonSRXWrapper driveLeftTalonWrapperPercentOutput2;
	public static TalonSRXWrapper driveLeftTalonWrapperPercentOutput3;
	public static TalonSRXWrapper driveLeftTalonWrapperPercentOutput4;

	public static TalonSRXWrapper driveRightTalonWrapperPercentOutput1;
	public static TalonSRXWrapper driveRightTalonWrapperPercentOutput2;
	public static TalonSRXWrapper driveRightTalonWrapperPercentOutput3;
	public static TalonSRXWrapper driveRightTalonWrapperPercentOutput4;

	public static SpeedControllerWrapper driveLeftCurrent;
	public static SpeedControllerWrapper driveLeftPercentOutput;

	public static SpeedControllerWrapper driveRightCurrent;
	public static SpeedControllerWrapper driveRightPercentOutput;

	public static Encoder driveLeftEncoder;
	public static Encoder driveRightEncoder;

	public static EncoderWrapperRateAndDistance driveLeftEncoderWrapperDistance;
	public static EncoderWrapperRateAndDistance driveRightEncoderWrapperDistance;
	public static EncoderWrapperRateAndDistance driveLeftEncoderWrapperRate;
	public static EncoderWrapperRateAndDistance driveRightEncoderWrapperRate;

	public static PigeonIMU gyro;
	public static PigeonWrapperRateAndAngle gyroRateWrapper;
	public static PigeonWrapperRateAndAngle gyroAngleWrapper;


	//ELEVATOR
	public static TalonSRX elevatorTalon1;
	public static TalonSRX elevatorTalon2;

	public static TalonSRXWrapper elevatorTalonWrapperCurrent1;
	public static TalonSRXWrapper elevatorTalonWrapperCurrent2;

	public static TalonSRXWrapper elevatorTalonWrapperPercentOutput1;
	public static TalonSRXWrapper elevatorTalonWrapperPercentOutput2;

	public static SpeedControllerWrapper elevatorPercentOutput;
	public static SpeedControllerWrapper elevatorCurrent;

	public static Encoder elevatorEncoder;
	
	public static EncoderWrapperRateAndDistance elevatorEncoderWrapperDistance;


	//CARGO INTAKE ARM
	public static TalonSRX cargoArmTalon;

	public static TalonSRXWrapper cargoArmTalonWrapperCurrent;
	public static TalonSRXWrapper cargoArmTalonWrapperPercentOutput;

	public static SpeedControllerWrapper cargoArmCurrent;
	public static SpeedControllerWrapper cargoArmPercentOutput;

	public static Encoder cargoArmEncoder;

	public static EncoderWrapperRateAndDistance cargoArmEncoderWrapperDistance;

	public static DigitalInput cargoArmLimitSwitchDown;
	public static DigitalInput cargoArmLimitSwitchUp;

	
	//CARGO INTAKE ROLLERS
	public static TalonSRX cargoRollersTalon;

	public static TalonSRXWrapper cargoRollersWrapperCurrent;
	public static TalonSRXWrapper cargoRollersWrapperPercentOutput;

	public static SpeedControllerWrapper cargoRollersCurrent;
	public static SpeedControllerWrapper cargoRollersPercentOutput;


	//HATCH INTAKE 
	public static Solenoid liftSolenoidIn, liftSolenoidOut;
	public static Solenoid pushersSolenoidIn, pushersSolenoidOut;
	public static Solenoid slideSolenoidIn, slideSolenoidOut;
	public static Solenoid hookSolenoidIn, hookSolenoidOut;



	//SUBSYSTEMS
	public static DriveTrain driveTrain;
	public static Elevator elevator;
	public static CargoArm cargoArm;
	public static CargoRollers cargoRollers;
	public static HatchIntake hatchIntake;

	// For example to map the left and right motors, you could define the
	// following variables to use with your drivetrain subsystem.
	// public static int leftMotor = 1;
	// public static int rightMotor = 2;

	// If you are using multiple modules, make sure to define both the port
	// number and the module. For example you with a rangefinder:
	// public static int rangefinderPort = 1;
	// public static int rangefinderModule = 1;
	public static void init() {

		//DRIVE TRAIN
		driveLeftTalon1 = new TalonSRX(driveLeftTalonPort1);
		driveLeftTalon2 = new TalonSRX(driveLeftTalonPort2);
		driveLeftTalon3 = new TalonSRX(driveLeftTalonPort3);
		driveLeftTalon4 = new TalonSRX(driveLeftTalonPort4);

		driveRightTalon1 = new TalonSRX(driveRightTalonPort1);
		driveRightTalon2 = new TalonSRX(driveRightTalonPort2);
		driveRightTalon3 = new TalonSRX(driveRightTalonPort3);
		driveRightTalon4 = new TalonSRX(driveRightTalonPort4);

		driveLeftTalon1.configContinuousCurrentLimit(7);
		driveLeftTalon2.configContinuousCurrentLimit(7);
		driveLeftTalon3.configContinuousCurrentLimit(7);
		driveLeftTalon4.configContinuousCurrentLimit(7);

		driveRightTalon1.configContinuousCurrentLimit(7);
		driveRightTalon2.configContinuousCurrentLimit(7);
		driveRightTalon3.configContinuousCurrentLimit(7);
		driveRightTalon4.configContinuousCurrentLimit(7);

		driveLeftTalon1.configPeakCurrentLimit(7);
		driveLeftTalon2.configPeakCurrentLimit(7);
		driveLeftTalon3.configPeakCurrentLimit(7);
		driveLeftTalon4.configPeakCurrentLimit(7);

		driveRightTalon1.configPeakCurrentLimit(7);
		driveRightTalon2.configPeakCurrentLimit(7);
		driveRightTalon3.configPeakCurrentLimit(7);
		driveRightTalon4.configPeakCurrentLimit(7);

		driveLeftTalon1.enableCurrentLimit(true);
		driveLeftTalon2.enableCurrentLimit(true);
		driveLeftTalon3.enableCurrentLimit(true);
		driveLeftTalon4.enableCurrentLimit(true);

		driveRightTalon1.enableCurrentLimit(true);
		driveRightTalon2.enableCurrentLimit(true);
		driveRightTalon3.enableCurrentLimit(true);
		driveRightTalon4.enableCurrentLimit(true);




		gyroTalon = new TalonSRX(gyroTalonPort);

		driveLeftTalonWrapperCurrent1 = new TalonSRXWrapper(ControlMode.Current, driveLeftTalon1);
		driveLeftTalonWrapperCurrent2 = new TalonSRXWrapper(ControlMode.Current, driveLeftTalon2);
		driveLeftTalonWrapperCurrent3 = new TalonSRXWrapper(ControlMode.Current, driveLeftTalon3);
		driveLeftTalonWrapperCurrent4 = new TalonSRXWrapper(ControlMode.Current, driveLeftTalon4);

		driveRightTalonWrapperCurrent1 = new TalonSRXWrapper(ControlMode.Current, driveRightTalon1);
		driveRightTalonWrapperCurrent2 = new TalonSRXWrapper(ControlMode.Current, driveRightTalon2);
		driveRightTalonWrapperCurrent3 = new TalonSRXWrapper(ControlMode.Current, driveRightTalon3);
		driveRightTalonWrapperCurrent4 = new TalonSRXWrapper(ControlMode.Current, driveRightTalon4);

		driveLeftTalonWrapperPercentOutput1 = new TalonSRXWrapper(ControlMode.PercentOutput, driveLeftTalon1);
		driveLeftTalonWrapperPercentOutput2 = new TalonSRXWrapper(ControlMode.PercentOutput, driveLeftTalon2);
		driveLeftTalonWrapperPercentOutput3 = new TalonSRXWrapper(ControlMode.PercentOutput, driveLeftTalon3);
		driveLeftTalonWrapperPercentOutput4 = new TalonSRXWrapper(ControlMode.PercentOutput, driveLeftTalon4);

		driveRightTalonWrapperPercentOutput1 = new TalonSRXWrapper(ControlMode.PercentOutput, driveRightTalon1);
		driveRightTalonWrapperPercentOutput2 = new TalonSRXWrapper(ControlMode.PercentOutput, driveRightTalon2);
		driveRightTalonWrapperPercentOutput3 = new TalonSRXWrapper(ControlMode.PercentOutput, driveRightTalon3);
		driveRightTalonWrapperPercentOutput4 = new TalonSRXWrapper(ControlMode.PercentOutput, driveRightTalon4);


		TalonSRXWrapper[] driveLeftTalonsCurrent = {driveLeftTalonWrapperCurrent1, driveLeftTalonWrapperCurrent2, driveLeftTalonWrapperCurrent3, driveLeftTalonWrapperCurrent4};
		TalonSRXWrapper[] driveLeftTalonsPercentOutput = {driveLeftTalonWrapperPercentOutput1, driveLeftTalonWrapperPercentOutput2, driveLeftTalonWrapperPercentOutput3, driveLeftTalonWrapperPercentOutput4};

		TalonSRXWrapper[] driveRightTalonsCurrent = {driveRightTalonWrapperCurrent1, driveRightTalonWrapperCurrent2, driveRightTalonWrapperCurrent3, driveRightTalonWrapperCurrent4};
		TalonSRXWrapper[] driveRightTalonsPercentOutput = {driveRightTalonWrapperPercentOutput1, driveRightTalonWrapperPercentOutput2, driveRightTalonWrapperPercentOutput3, driveRightTalonWrapperPercentOutput4};
		

		driveLeftCurrent = new SpeedControllerWrapper(driveLeftTalonsCurrent);
		driveLeftPercentOutput = new SpeedControllerWrapper(driveLeftTalonsPercentOutput);

		driveRightCurrent = new SpeedControllerWrapper(driveRightTalonsCurrent);
		driveRightPercentOutput = new SpeedControllerWrapper(driveRightTalonsPercentOutput);

		driveLeftCurrent.setInverted(true);
		driveLeftPercentOutput.setInverted(true);

		driveLeftTalonWrapperCurrent3.setInverted(true);
		driveLeftTalonWrapperPercentOutput3.setInverted(true);

		driveLeftEncoder = new Encoder(driveLeftEncoderPort1, driveLeftEncoderPort2);
		driveRightEncoder = new Encoder(driveRightEncoderPort1, driveRightEncoderPort2);
		
		driveLeftEncoderWrapperDistance = new EncoderWrapperRateAndDistance(driveLeftEncoder, PIDSourceType.kDisplacement);
		driveRightEncoderWrapperDistance = new EncoderWrapperRateAndDistance(driveRightEncoder, PIDSourceType.kDisplacement);
		driveLeftEncoderWrapperRate = new EncoderWrapperRateAndDistance(driveLeftEncoder, PIDSourceType.kRate);
		driveRightEncoderWrapperRate = new EncoderWrapperRateAndDistance(driveRightEncoder, PIDSourceType.kRate);

		gyro = new PigeonIMU(gyroTalon);
		gyroAngleWrapper = new PigeonWrapperRateAndAngle(gyro, PIDSourceType.kDisplacement, Units.RADS);
		gyroRateWrapper = new PigeonWrapperRateAndAngle(gyro, PIDSourceType.kRate, Units.RADS);


		//ELEVATOR
		elevatorTalon1 = new TalonSRX(elevatorTalonPort1);
		elevatorTalon2 = new TalonSRX(elevatorTalonPort2);

		elevatorTalonWrapperCurrent1 = new TalonSRXWrapper(ControlMode.Current, elevatorTalon1);
		elevatorTalonWrapperCurrent2 = new TalonSRXWrapper(ControlMode.Current, elevatorTalon2);
		
		elevatorTalonWrapperPercentOutput1 = new TalonSRXWrapper(ControlMode.PercentOutput, elevatorTalon1);
		elevatorTalonWrapperPercentOutput2 = new TalonSRXWrapper(ControlMode.PercentOutput, elevatorTalon2);

		TalonSRXWrapper[] elevatorTalonsCurrent = {elevatorTalonWrapperCurrent1, elevatorTalonWrapperCurrent2};
		TalonSRXWrapper[] elevatorTalonsPercentOutput = {elevatorTalonWrapperPercentOutput1, elevatorTalonWrapperPercentOutput2};

		elevatorCurrent = new SpeedControllerWrapper(elevatorTalonsCurrent);
		elevatorPercentOutput = new SpeedControllerWrapper(elevatorTalonsPercentOutput);

		elevatorEncoder = new Encoder(elevatorEncoderPort1, elevatorEncoderPort2);

		elevatorEncoderWrapperDistance = new EncoderWrapperRateAndDistance(elevatorEncoder, PIDSourceType.kDisplacement);
		
		elevatorEncoder.setDistancePerPulse((DRUM_RADIUS*Math.PI*2/250));
		elevatorEncoderWrapperDistance.setGearRatio(2);
		elevatorEncoder.setSamplesToAverage(10);



		//CARGO INTAKE ARM
		cargoArmTalon = new TalonSRX(cargoArmTalonPort);

		cargoArmTalon.setInverted(true);

		cargoArmTalonWrapperCurrent = new TalonSRXWrapper(ControlMode.Current, cargoArmTalon);
		cargoArmTalonWrapperPercentOutput = new TalonSRXWrapper(ControlMode.PercentOutput, cargoArmTalon);

		cargoArmCurrent = new SpeedControllerWrapper(cargoArmTalonWrapperCurrent);
		cargoArmPercentOutput = new SpeedControllerWrapper(cargoArmTalonWrapperPercentOutput);

		cargoArmEncoder = new Encoder(cargoArmEncoderPort1, cargoArmEncoderPort2);

		cargoArmEncoderWrapperDistance = new EncoderWrapperRateAndDistance(cargoArmEncoder, PIDSourceType.kDisplacement);

		cargoArmEncoder.setDistancePerPulse(2 * Math.PI / 1024);

		cargoArmLimitSwitchUp = new DigitalInput(8);
		cargoArmLimitSwitchDown = new DigitalInput(9);


		//CARGO INTAKE ROLLERS
		cargoRollersTalon = new TalonSRX(cargoRollersTalonPort);

		cargoRollersWrapperCurrent = new TalonSRXWrapper(ControlMode.Current, cargoRollersTalon);
		cargoRollersWrapperPercentOutput = new TalonSRXWrapper(ControlMode.PercentOutput, cargoRollersTalon);

		cargoRollersCurrent = new SpeedControllerWrapper(cargoRollersWrapperCurrent);
		cargoRollersPercentOutput = new SpeedControllerWrapper(cargoRollersWrapperPercentOutput);

		cargoRollersCurrent.setInverted(true);
		cargoRollersPercentOutput.setInverted(true);


		//HATCH INTAKE
		liftSolenoidIn = new Solenoid(liftSolenoidPortIn);
		liftSolenoidOut = new Solenoid(liftSolenoidPortOut);
		
		pushersSolenoidIn = new Solenoid(pushersSolenoidPortIn);
		pushersSolenoidOut = new Solenoid(pushersSolenoidPortOut);
		
		slideSolenoidIn = new Solenoid(slideSolenoidPortIn);
		slideSolenoidOut = new Solenoid(slideSolenoidPortOut);
		
		hookSolenoidIn = new Solenoid(hookSolenoidPortIn);
		hookSolenoidOut = new Solenoid(hookSolenoidPortOut);



		//SUBSYSTEMS
		driveTrain = new DriveTrain();
		elevator = new Elevator();
		cargoArm = new CargoArm();
		cargoRollers = new CargoRollers();
		hatchIntake = new HatchIntake();
		

	}

	public static void updateConstants() {
		driveTrain.updateConstants();
		elevator.updateConstants();
		cargoArm.updateConstants();
	}

	
}
