/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2485.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.cameraserver.CameraServer;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.vision.VisionThread;


import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.vision.VisionRunner;


import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.FastMath;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.GripPipeline;
import org.usfirst.frc.team2485.util.RampRate;
import org.usfirst.frc.team2485.util.SurajPipeline;
import org.usfirst.frc.team2485.robot.commandGroups.SandstormAuto;
import org.usfirst.frc.team2485.robot.commands.CancelCommand;
import org.usfirst.frc.team2485.robot.commands.CargoArmWithControllers;
import org.usfirst.frc.team2485.robot.commands.DriveStraight;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;
import org.usfirst.frc.team2485.robot.commands.ElevatorWithControllers;
import org.usfirst.frc.team2485.robot.commands.Lift;
import org.usfirst.frc.team2485.robot.commands.SetAngle;
import org.usfirst.frc.team2485.robot.commands.SetArmPosition;
import org.usfirst.frc.team2485.robot.commands.SetRollers;
import org.usfirst.frc.team2485.robot.subsystems.CargoArm;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;





/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */

public class Robot extends TimedRobot {

	public static final int IMG_WIDTH = 120;
	public static final int IMG_HEIGHT = 90;
	
	private VisionThread visionThread;
	public static double centerX = 0.0;
	public static ArrayList<Double> samples;
	public static boolean doneCollecting = false;

	public static Pair[] controlPoints;
	public Pair endpoint;

	


	private static AutoPath path;


	private final Object imgLock = new Object();
	
	
	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	public static boolean collectingSamples;

	public static boolean contoursVisible;
	
	boolean ejecting = false;

	public static Command auto;

	public CvSource output;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
		FastMath.init();
		ConstantsIO.init();
		RobotMap.init();
		OI.init();

		SandstormAuto.init(false); 
		auto = new SandstormAuto();

		// UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		// camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
		
		// output = CameraServer.getInstance().putVideo("Processed: ", 640, 480);

		
		// VisionThread visionThread = new VisionThread(camera, new SurajPipeline(), pipeline -> {
		// 	output.putFrame(pipeline.cvRectangleOutput());

		// });
		// visionThread.start();

	
	
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
	
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString code to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional commands to the
	 * chooser code above (like the commented example) or additional comparisons
	 * to the switch structure below with additional strings & commands.
	 */
	@Override
	public void autonomousInit() {
		ConstantsIO.init();
		RobotMap.driveTrain.updateConstants();
		RobotMap.hatchIntake.slideIn();
		RobotMap.hatchIntake.hookIn();
		RobotMap.hatchIntake.retractPushers();
		RobotMap.hatchIntake.lift();
		RobotMap.cargoArmEncoder.reset();
		RobotMap.elevatorEncoder.reset();
		RobotMap.driveLeftEncoder.reset();
		RobotMap.driveRightEncoder.reset();
		RobotMap.gyroAngleWrapper.reset();
		RobotMap.gyroRateWrapper.reset();
		
		
		RobotMap.driveLeftTalon1.clearStickyFaults();
		RobotMap.driveLeftTalon2.clearStickyFaults();
		RobotMap.driveLeftTalon3.clearStickyFaults();
		RobotMap.driveLeftTalon4.clearStickyFaults();
		
		RobotMap.driveRightTalon1.clearStickyFaults();
		RobotMap.driveRightTalon2.clearStickyFaults();
		RobotMap.driveRightTalon3.clearStickyFaults();
		RobotMap.driveRightTalon4.clearStickyFaults();

		RobotMap.driveTrain.enablePID(true);
		RobotMap.cargoArm.enablePID(true);
		SandstormAuto.init(true); //true is left side, false for right
		// Scheduler.getInstance().add(new SetArmPosition(0));
		//Scheduler.getInstance().add(new DriveWithControllers());
		// Scheduler.getInstance().add(new SandstormAuto());
		

		// Scheduler.getInstance().add(new Lift(true));
		
		// RobotMap.driveTrain.anglePID.disable();
		// RobotMap.driveTrain.angVelPID.enable();
		// RobotMap.driveTrain.leftMotorSetter.enable();
		// RobotMap.driveTrain.rightMotorSetter.enable();
		// RobotMap.driveTrain.angleSetpointTN.setOutput(Math.PI/2);
		//RobotMap.driveTrain.setVelocities(0, 0.1);
		// RobotMap.driveTrain.enablePID(false);
		// Scheduler.getInstance().add(auto);

		// RobotMap.driveTrain.distancePID.enable();
		// RobotMap.driveTrain.velocityPID.enable();
		// RobotMap.driveTrain.leftMotorSetter.enable();
		// RobotMap.driveTrain.rightMotorSetter.enable();
		// RobotMap.driveTrain.distanceSetpointTN.setOutput(40);
		




		//  RobotMap.driveTrain.setVelocities(10, 0.2);

		// RobotMap.driveTrain.anglePID.enable();
		// RobotMap.driveTrain.leftMotorSetter.enable();
		// RobotMap.driveTrain.rightMotorSetter.enable();
		
		// RobotMap.driveTrain.angleSetpointTN.setOutput(Math.PI/2);

		
		// RobotMap.driveTrain.enablePID(true);

		//RobotMap.driveTrain.distanceSetpointTN.setOutput(100);	
		//RobotMap.driveTrain.setVelocities(30, 0);
		//RobotMap.elevatorCurrent.set(0.2);


		//RobotMap.driveTrain.enablePID(true);
		//  Pair[] controlPoints = { new Pair( 0, 0), new Pair(0, 214), new Pair(-35.5, 214) };
		
	// 	// double[] dists = { 90.0, };
	// AutoPath path = new AutoPath(AutoPath.getPointsForBezier(1000, controlPoints[0], controlPoints[1], controlPoints[2]));
	// 	 Scheduler.getInstance().add(new DriveTo(path, 50, false, 15000, false, true));

		//Scheduler.getInstance().add(auto);
		//controlPoints = RobotMap.driveTrain.generateControlPoints(100, 70, 90);
		//endpoint = RobotMap.driveTrain.getAutoAlignEndpoint(100, 70, 90);

		
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */


		// schedule the autonomous command (example)
	
	




		
		// RobotMap.driveTrain.distancePID.setSetpoint(RobotMap.lidar.getDistance() - 10);

		//RobotMap.elevator.setElevatorVelocity(15);

		// Scheduler.getInstance().add(new SandstormAuto());
		// Scheduler.getInstance().add(new DriveTo(new AutoPath(AutoPath.getPointsForBezier(2000, new Pair(0.0, 0.0), new Pair(0, 165.5),
		// 	new Pair(-22.5, 165.5))), 60, false, 15000, false, true));

		// Scheduler.getInstance().add(new SetAngle(Math.PI/2, 10000));
		// Scheduler.getInstance().add(new DriveStraight(70, 2000));

		// AutoPath path = AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);
			
		// Scheduler.getInstance().add(new DriveTo(path, 60, true, 15000, false, true));

		//RobotMap.driveTrain.angleOutputTN.setOutput(0.2);

		//Scheduler.getInstance().add(new DriveStraight(100, 1000000));
		
	


	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		

		
		Scheduler.getInstance().run();
		updateSmartDashboard();

		// if(OI.suraj.getRawButton(OI.XBOX_BACK_BUTTON)) {
		// 	Scheduler.getInstance().add(new CancelCommand(auto));
		// } 

		// if(OI.getDriveThrottle() != 0) {
		// 	Scheduler.getInstance().add(new CancelCommand(auto));
		// }

		// if(OI.getDriveSteering() != 0) {
		// 	Scheduler.getInstance().add(new CancelCommand(auto));

		// }

		// RobotMap.elevatorWrapperCurrent.set(1);
		//RobotMap.cargoArm.distanceSetpointTN.setOutput(1);


	

		// RobotMap.elevatorTalonWrapperCurrent1.set(15);
		// RobotMap.elevatorTalonWrapperCurrent2.set(15);

		//RobotMap.driveTrain.setAngle(Math.PI/2);
		
		

		
		// RobotMap.driveTrain.setHighLowCurrent(-0.5, -0.5, -0.5, -0.5, 1000);
		
	}

	@Override
	public void teleopInit() {
		ConstantsIO.init();
		RobotMap.compressor.setClosedLoopControl(true);
		RobotMap.cargoArmEncoder.reset();
		RobotMap.elevatorEncoder.reset();
		RobotMap.updateConstants();
		RobotMap.hatchIntake.slideIn();
		RobotMap.hatchIntake.hookIn();
		RobotMap.hatchIntake.retractPushers();
		RobotMap.hatchIntake.stow();
		RobotMap.driveTrain.enablePID(false);
		//RobotMap.elevator.enablePID(true);
		//RobotMap.elevator.distanceSetpointTN.setOutput(33.5);
		RobotMap.cargoArm.enablePID(false);
		// RobotMap.cargoArm.distanceSetpointTN.setOutput(1);
		//RobotMap.elevator.enablePID(true);
		//RobotMap.elevator.distanceSetpointTN.setOutput(ElevatorLevel.ROCKET_LEVEL_TWO.getPosition());
		//RobotMap.cargoArm.enablePID(false);
		//RobotMap.cargoArm.distanceSetpointTN.setOutput(1);
		//Scheduler.getInstance().add(new SetRollers(0));
		
		// Scheduler.getInstance().add(new SetArmPosition(CargoArm.TOP_POSITION));
		// Scheduler.getInstance().add(new SetArmPosition(0));

		// Scheduler.getInstance().add(new CancelCommand(auto));
		//Scheduler.getInstance().add(new CargoArmWithControllers());
		
		// UsbCamera jevoisCam = CameraServer.getInstance().startAutomaticCapture();
		// jevoisCam.setVideoMode(PixelFormat.kYUYV, 160, 120, 30);


		
		

		//Scheduler.getInstance().add(new SetArmPosition(CargoArm.TOP_POSITION));

		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		//Scheduler.getInstance().add(new DriveWithControllers());

	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
		
		// if(OI.suraj.getRawAxis(OI.XBOX_LTRIGGER_PORT) > 0.2) {
		// 	Scheduler.getInstance().add(new SetRollers(-0.4));
		// 	ejecting = true;
		// } else if (ejecting) {
		// 	ejecting = false;
		// 	Scheduler.getInstance().add(new SetRollers(0));
		// }


		if (RobotMap.cargoArmLimitSwitchUp.get()) {
			RobotMap.cargoArmEncoder.reset();
		}
		


	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		RobotMap.compressor.setClosedLoopControl(true);
		updateSmartDashboard();
	}

	public void updateSmartDashboard() {

		SmartDashboard.putString("ConstantsIO Last Modified: ", ConstantsIO.lastModified);

		SmartDashboard.putNumber("Elevator Distance PID Setpoint: ", RobotMap.elevator.distancePID.getSetpoint());
		SmartDashboard.putNumber("Elevator Distance PID Error: ", RobotMap.elevator.distancePID.getError());
		SmartDashboard.putNumber("Elevator Setpoint TN: ", RobotMap.elevator.distanceSetpointTN.getOutput());
		SmartDashboard.putNumber("Elevator Setpoint Ramped TN: ", RobotMap.elevator.distanceSetpointRampedTN.getOutput());
		SmartDashboard.putBoolean("Elevator Setpoint Ramp Rate Enabled: ", RobotMap.elevator.distanceSetpointRampRate.isEnabled());
		SmartDashboard.putBoolean("Elevator Distance PID Enabled: ", RobotMap.elevator.distancePID.isEnabled());
		SmartDashboard.putNumber("Elevator Position:", RobotMap.elevatorEncoderWrapperDistance.pidGet());
		SmartDashboard.putNumber("Elevator Hatch Placement: ", RobotMap.elevatorEncoderWrapperDistance.pidGet() - 3);
		SmartDashboard.putNumber("Elevator Output Current: ", RobotMap.elevatorTalon1.getOutputCurrent());
		SmartDashboard.putNumber("Elevator Encoder: ", RobotMap.elevatorEncoderWrapperDistance.pidGet());
		SmartDashboard.putNumber("Elevator Distance Output TN:", RobotMap.elevator.distanceOutputTN.getOutput());
		SmartDashboard.putNumber("Elevator AntiGravityPIDSource:", RobotMap.elevator.elevatorAntiGravityPIDSource.pidGet());
		SmartDashboard.putNumber("Elevator antiGravity Output TN:", RobotMap.elevator.antiGravityOutputTN.getOutput());
		SmartDashboard.putNumber("ConstantsIO.kF_elevatorAntiGravity:", ConstantsIO.kF_elevatorAntiGravity);
		SmartDashboard.putNumber("distanceOutputPIDSource:", RobotMap.elevator.distanceOutputPIDSource.pidGet());


		SmartDashboard.putNumber("Cargo Arm Distance PID Error: ", RobotMap.cargoArm.distancePID.getError());
		SmartDashboard.putNumber("Cargo Arm Setpoint TN: ", RobotMap.cargoArm.distanceSetpointTN.getOutput());
		SmartDashboard.putBoolean("Cargo Arm Distance PID Enabled: ", RobotMap.cargoArm.distancePID.isEnabled());
		SmartDashboard.putNumber("Cargo Arm Position: ", RobotMap.cargoArmEncoderWrapperDistance.pidGet());	
		SmartDashboard.putNumber("Cargo Arm PID Source Output: ", RobotMap.cargoArm.distanceOutputPIDSource.pidGet());
		SmartDashboard.putBoolean("Cargo Arm on position:", RobotMap.cargoArm.distancePID.isOnTarget());
		SmartDashboard.putBoolean("Cargo Arm Up Limit Switch: ", RobotMap.cargoArmLimitSwitchUp.get());
		SmartDashboard.putBoolean("Cargo Arm Down Limit Switch: ", RobotMap.cargoArmLimitSwitchDown.get());
		SmartDashboard.putNumber("Cargo Arm Output Current: ", RobotMap.cargoArmTalon.getOutputCurrent());
		SmartDashboard.putNumber("Cargo Arm Distance PID Setpoint Please: ", RobotMap.cargoArm.distancePID.getSetpoint());
		SmartDashboard.putNumber("Cargo Arm Failsafe TN: ", RobotMap.cargoArm.failsafeTN.getOutput());
		SmartDashboard.putNumber("Cargo Arm Error: ", RobotMap.cargoArm.distancePID.getError());


		SmartDashboard.putNumber("Cargo Roller Current", RobotMap.cargoRollersTalon.getOutputCurrent());
		SmartDashboard.putBoolean("Cargo Intaken: ", RobotMap.cargoRollers.intaken);

		SmartDashboard.putNumber("Drive Train Distance Setpoint: ", RobotMap.driveTrain.distanceSetpointTN.pidGet());
		SmartDashboard.putNumber("Drive Train Angle Setpoint: ", RobotMap.driveTrain.angleSetpointTN.pidGet());
		SmartDashboard.putNumber("DT Control Point X: ", RobotMap.driveTrain.generateSandstormControlPoints(true)[0].getX());
		SmartDashboard.putNumber("DT Control Point Y: ", RobotMap.driveTrain.generateSandstormControlPoints(true)[0].getY());
		SmartDashboard.putNumber("DT Endpoint X: ", RobotMap.driveTrain.getSandstormEndpoint(true).getX());
		SmartDashboard.putNumber("DT Endpoint Y: ", RobotMap.driveTrain.getSandstormEndpoint(true).getY());
		SmartDashboard.putBoolean("Drive Train Distance PID Enabled: ", RobotMap.driveTrain.distancePID.isEnabled());
		SmartDashboard.putBoolean("Drive Train Angle PID Enabled: ", RobotMap.driveTrain.anglePID.isEnabled());
		SmartDashboard.putNumber("Drive Train Talon Current: ", RobotMap.driveLeftTalon1.getOutputCurrent());
		SmartDashboard.putNumber("Drive Train Distance Output: ", RobotMap.driveTrain.distanceOutputTN.getOutput());
		SmartDashboard.putNumber("Drive Train Velocity Output: ", RobotMap.driveTrain.velocityOutputTN.getOutput());
		SmartDashboard.putNumber("Drive Train Velocity PID Setpoint: ", RobotMap.driveTrain.velocityPID.getSetpoint());
		SmartDashboard.putNumber("Drive Train Velocity PID Source: ", RobotMap.driveTrain.velocityPIDSource.pidGet());
		SmartDashboard.putNumber("Drive Train Distance Output TN: ", RobotMap.driveTrain.distanceOutputTN.getOutput());
		SmartDashboard.putNumber("Drive Train Distance Error: ", RobotMap.driveTrain.distancePID.getError());
		SmartDashboard.putNumber("Drive Train Velocity Error: ", RobotMap.driveTrain.velocityPID.getError());
		SmartDashboard.putNumber("DriveTrain Distance Source: ", RobotMap.driveTrain.distancePIDSource.pidGet());
		SmartDashboard.putNumber("Drive Train Left Encoder Wrapper Dist: ", RobotMap.driveLeftEncoderWrapperDistance.pidGet());
		SmartDashboard.putNumber("Drive Train Right Encoder Wrapper Dist: ", RobotMap.driveRightEncoderWrapperDistance.pidGet());
		SmartDashboard.putNumber("Drive Train Ang Vel Error: ", RobotMap.driveTrain.angVelPID.getError());
		SmartDashboard.putNumber("Drive Train Angle Error: ", RobotMap.driveTrain.anglePID.getError());
		SmartDashboard.putNumber("Cargo Arm Talon Output", RobotMap.cargoArmTalon.getMotorOutputPercent());
		SmartDashboard.putNumber("Drive Train Ang Vel Output", RobotMap.driveTrain.angVelOutputTN.getOutput());
		SmartDashboard.putNumber("kP Ang Vel", ConstantsIO.kP_DriveAngVel);
		SmartDashboard.putNumber("kP Ang Vel public version", RobotMap.driveTrain.angVelPID.kP);
		SmartDashboard.putNumber("Gyro Value:", RobotMap.gyroAngleWrapper.pidGet());
		SmartDashboard.putBoolean("Ang Vel Enabled: ", RobotMap.driveTrain.angVelPID.isEnabled());
		SmartDashboard.putNumber("Drive Talon Left 1 Current:", RobotMap.driveLeftTalon1.getOutputCurrent());
		SmartDashboard.putNumber("Drive Talon Left 2 Current:", RobotMap.driveLeftTalon2.getOutputCurrent());
		SmartDashboard.putNumber("Drive Talon Left 3 Current:", RobotMap.driveLeftTalon3.getOutputCurrent());
		SmartDashboard.putNumber("Drive Talon Left 4 Current:", RobotMap.driveLeftTalon4.getOutputCurrent());
		SmartDashboard.putNumber("Drive Talon Right 1 Current:", RobotMap.driveRightTalon1.getOutputCurrent());
		SmartDashboard.putNumber("Drive Talon Right 2 Current:", RobotMap.driveRightTalon2.getOutputCurrent());
		SmartDashboard.putNumber("Drive Talon Right 3 Current:", RobotMap.driveRightTalon3.getOutputCurrent());
		SmartDashboard.putNumber("Drive Talon Right 4 Current:", RobotMap.driveRightTalon4.getOutputCurrent());
		SmartDashboard.putBoolean("Drive Train Velocity Enabled: ", RobotMap.driveTrain.velocityPID.isEnabled());
		SmartDashboard.putBoolean("Lift Up: ", RobotMap.liftSolenoidOut.get());
		SmartDashboard.putNumber("Suraj RYStick output: ", OI.getArmManual());
		SmartDashboard.putNumber("Arm Ramped Setpoint:", RobotMap.cargoArm.distanceSetpointRampedTN.getOutput());
		SmartDashboard.putNumber("up ramp arm:", ConstantsIO.armDistanceSetpointUpRamp);
		SmartDashboard.putNumber("Cargo Arm output current", RobotMap.cargoArmTalon.getOutputCurrent());
		SmartDashboard.putNumber("Cargo Arm Distance Output TN ", RobotMap.cargoArm.distanceOutputTN.getOutput());
		SmartDashboard.putNumber("Cargo Arm Ramped Distance Setpoint TN: ", RobotMap.cargoArm.distanceSetpointRampedTN.getOutput());
		SmartDashboard.putNumber("Distance Output PID Source: ", RobotMap.cargoArm.distanceOutputPIDSource.pidGet());
		SmartDashboard.putNumber("ELevator Distance Output TN: ", RobotMap.elevator.distanceOutputTN.getOutput());

		SmartDashboard.putNumber("Cargo Arm Ramped TN: ", RobotMap.cargoArm.distanceSetpointRampedTN.getOutput());

	}
	
	

	
}
