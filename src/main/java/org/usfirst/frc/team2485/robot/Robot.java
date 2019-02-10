/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2485.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.wpilibj.CameraServer;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.VisionThread;

import org.usfirst.frc.team2485.robot.commandGroups.Docking;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;
import org.usfirst.frc.team2485.robot.commands.ElevatorMove;
import org.usfirst.frc.team2485.robot.subsystems.DriveTrain;
import  org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.FastMath;
import  org.usfirst.frc.team2485.util.Pipeline;
import org.usfirst.frc.team2485.util.WarlordsPIDController;
import  org.usfirst.frc.team2485.util.AutoPath;


import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;




/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */

public class Robot extends TimedRobot {
	
	private static final int IMG_WIDTH = 128;
	private static final int IMG_HEIGHT = 96;
	
	private VisionThread visionThread;
	public static double centerX = 0.0;

	public Pair[] controlPoints;
	public Pair endpoint;


	private static AutoPath path;


	UsbCamera camera;
	private final Object imgLock = new Object();
	
	
	Command m_autonomousCommand;
	SendableChooser<Command> m_chooser = new SendableChooser<>();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// chooser.addObject("My Auto", new MyAutoCommand());
		SmartDashboard.putData("Auto mode", m_chooser);
		ConstantsIO.init();
		RobotMap.init();
		OI.init();
		FastMath.init();
		RobotMap.driveTrain.updateConstants();
		RobotMap.elevator.updateConstants();

		RobotMap.driveLeftEncoder.reset();
		RobotMap.driveRightEncoder.reset();
		RobotMap.gyroRateWrapper.reset();
		RobotMap.gyroAngleWrapper.reset();

		controlPoints = RobotMap.driveTrain.generateControlPoints(RobotMap.lidar.pidGet(), 90, 110);
		endpoint = RobotMap.driveTrain.getAutoAlignEndpoint(RobotMap.lidar.pidGet(), 90, 110);

		path = new AutoPath (AutoPath.getPointsForBezier(2000, new Pair(0.0, 0.0), new Pair(0, 44.0), new Pair(53.5 - 6, 30.0), new Pair(53.5 - 6, 94)));


		// camera = CameraServer.getInstance().startAutomaticCapture();
		// camera.setVideoMode(PixelFormat.kYUYV, 160, 120, 30);
	    // camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
	    		
		//  visionThread = new VisionThread(camera, new Pipeline(), pipeline -> {
		//         if (!pipeline.filterContoursOutput().isEmpty()) {
		//             Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
		//             synchronized (imgLock) {
		//                 centerX = r.x + (r.width / 2);
		//                 System.out.println("Center X: " + centerX);
		//             }
		//         }
		//     });
		//     visionThread.start();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		RobotMap.driveTrain.updateConstants();
		RobotMap.elevator.updateConstants();
		RobotMap.driveTrain.enablePID(false);
		RobotMap.elevator.enablePID(false);
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
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
		RobotMap.driveLeftEncoder.reset();
		RobotMap.driveRightEncoder.reset();
		RobotMap.gyroRateWrapper.reset();
		RobotMap.gyroAngleWrapper.reset();
		RobotMap.driveTrain.updateConstants();
		RobotMap.elevator.updateConstants();
		RobotMap.elevatorEncoder.reset();
		m_autonomousCommand = m_chooser.getSelected();
		//RobotMap.driveTrain.enablePID(true);
		// Pair[] controlPoints = { new Pair( 35.6, -255.0), new Pair(0, -110.0), new Pair(0.0, 0.0) };
		
		// double[] dists = { 90.0, };
	   	// AutoPath path = AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);
		// Scheduler.getInstance().add(new DriveTo(path, 50, false, 20000, false, false));
		//controlPoints = RobotMap.driveTrain.generateControlPoints(100, 70, 90);
		//endpoint = RobotMap.driveTrain.getAutoAlignEndpoint(100, 70, 90);

		// Scheduler.getInstance().add(new Docking());
		
		/*
		 * String autoSelected = SmartDashboard.getString("Auto Selector",
		 * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
		 * = new MyAutoCommand(); break; case "Default Auto": default:
		 * autonomousCommand = new ExampleCommand(); break; }
		 */


		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.start();
		}

	




		
		//RobotMap.driveTrain.distancePID.setSetpoint(RobotMap.lidar.getDistance() - 20);

		//RobotMap.elevator.setElevatorVelocity(15);

		RobotMap.elevator.enablePID(true);
		RobotMap.elevator.setElevatorPosition(30);


	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();

		// RobotMap.elevatorWrapperCurrent.set(1);




	

		//RobotMap.elevatorTalonWrapperCurrent1.set(5);
		//RobotMap.elevatorTalonWrapperCurrent2.set(5);

		//RobotMap.driveTrain.setAngle(Math.PI/2);
		//RobotMap.driveTrain.setVelocities(30, 0.2);
		

		
		// RobotMap.driveTrain.setHighLowCurrent(-0.5, -0.5, -0.5, -0.5, 1000);
		
	}

	@Override
	public void teleopInit() {
		RobotMap.driveTrain.updateConstants();
		RobotMap.driveTrain.enablePID(false);
		RobotMap.driveTrain.updateConstants();
		RobotMap.elevator.updateConstants();

		Scheduler.getInstance().add(new ElevatorMove());


		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		//Scheduler.getInstance().add(new DriveWithControllers());

	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
		
		

	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		updateSmartDashboard();
	}

	public void updateSmartDashboard() {
		// SmartDashboard.putBoolean("Color Sensor", RobotMap.colorSensor.read(0x04,1,RobotMap.colorSensorOutput));
		SmartDashboard.putNumber("Left Talon 1: ", RobotMap.driveLeftTalon1.getOutputCurrent());
		SmartDashboard.putNumber("Right Talon 1: ", RobotMap.driveRightTalon1.getOutputCurrent());
		SmartDashboard.putNumber("Left Talon 2: ", RobotMap.driveLeftTalon2.getOutputCurrent());
		SmartDashboard.putNumber("Right Talon 2: ", RobotMap.driveRightTalon2.getOutputCurrent());
		SmartDashboard.putNumber("Velocity Error: ", RobotMap.driveTrain.velocityPID.getError());
		SmartDashboard.putNumber("Velocity Avg Error", RobotMap.driveTrain.velocityPID.getAvgError());
		SmartDashboard.putNumber("Velocity: ", ((RobotMap.driveLeftEncoderWrapperRate.pidGet()+RobotMap.driveRightEncoderWrapperRate.pidGet())/2));
		SmartDashboard.putNumber("Velocity PID Output: ", RobotMap.driveTrain.velocityTN.getOutput());
		SmartDashboard.putBoolean("Velocity is Enabled: ", RobotMap.driveTrain.velocityPID.isEnabled());
		SmartDashboard.putNumber("Setpoint", RobotMap.driveTrain.velocityPID.getSetpoint());
		SmartDashboard.putNumber("Velocity Setpoint TN Output", RobotMap.driveTrain.velocitySetpointTN.pidGet());
		SmartDashboard.putNumber("Left Current Source: ", RobotMap.driveTrain.leftCurrentPIDSource.pidGet());
		SmartDashboard.putNumber("Right Current Source: ", RobotMap.driveTrain.rightCurrentPIDSource.pidGet());
		SmartDashboard.putNumber("Left Motor Setter: ", RobotMap.driveTrain.leftMotorSetter.getSetpoint());
		SmartDashboard.putNumber("Right Motor Setter: ", RobotMap.driveTrain.rightMotorSetter.getSetpoint());

		SmartDashboard.putNumber("Velocity TN: ", RobotMap.driveTrain.velocityTN.pidGet());
		SmartDashboard.putNumber("Ang Vel TN", RobotMap.driveTrain.angVelTN.pidGet());
		SmartDashboard.putNumber("Velocity PID Source", RobotMap.driveTrain.velocityPIDSource.pidGet());

		SmartDashboard.putNumber("Angle Output", RobotMap.driveTrain.angVelSetpointTN.pidGet());

		SmartDashboard.putNumber("Angle Setpoint: ", RobotMap.driveTrain.anglePIDSource.pidGet());

		SmartDashboard.putNumber("Distance Error: ", RobotMap.driveTrain.distancePID.getAvgError());
		SmartDashboard.putNumber("Ang Vel avg error: ", RobotMap.driveTrain.angVelPID.getAvgError());
		SmartDashboard.putNumber("Ang Vel get error", RobotMap.driveTrain.angVelPID.getError());
		SmartDashboard.putNumber("gyro rate ", RobotMap.gyroRateWrapper.pidGet());


		SmartDashboard.putNumber("gyro angle ", RobotMap.gyroAngleWrapper.pidGet());
		SmartDashboard.putNumber("gyro angle error", RobotMap.driveTrain.anglePID.getError());


		SmartDashboard.putNumber("Distance Setpoint", RobotMap.driveTrain.distancePID.getSetpoint());
	
		SmartDashboard.putNumber("Velocity Left Enc", RobotMap.driveLeftEncoderWrapperRate.pidGet());
		SmartDashboard.putNumber("Velocity Right Enc", RobotMap.driveRightEncoderWrapperRate.pidGet());
		SmartDashboard.putNumber("Velocity setpoint TN", RobotMap.driveTrain.velocitySetpointTN.pidGet());

		SmartDashboard.putNumber("Ang Vel Setpoint", RobotMap.driveTrain.angVelTN.pidGet());
	
		SmartDashboard.putNumber("Lidar Value", RobotMap.lidar.getDistance());
		SmartDashboard.putNumber("Lidar Value 2", RobotMap.lidar.getDistance());

		SmartDashboard.putNumber("Control Point X 1", controlPoints[0].getX());
		SmartDashboard.putNumber("Control Point Y 1", controlPoints[0].getY());
		SmartDashboard.putNumber("Control Point X 2", controlPoints[1].getX());
		SmartDashboard.putNumber("Control Point Y 2", controlPoints[1].getY());
		SmartDashboard.putNumber("Control Points Length: ", controlPoints.length);

		SmartDashboard.putNumber("Endpoint X", endpoint.getX());
		SmartDashboard.putNumber("Endpoint Y", endpoint.getY());

		SmartDashboard.putNumber("Left Dist: ", RobotMap.driveLeftEncoderWrapperDistance.pidGet());
		SmartDashboard.putNumber("Right Dist: ", RobotMap.driveRightEncoderWrapperDistance.pidGet());

		SmartDashboard.putBoolean("Distance PID Enabled", RobotMap.driveTrain.distancePID.isEnabled());

		SmartDashboard.putNumber("Distance PID Source: ", RobotMap.driveTrain.distancePIDSource.pidGet());
		SmartDashboard.putNumber("kPath", ConstantsIO.kPath);


		SmartDashboard.putNumber("Angle cos", FastMath.cos(RobotMap.gyroAngleWrapper.pidGet()));

		SmartDashboard.putNumber("Surface angle", RobotMap.driveTrain.getThetaAlignmentLine());

		SmartDashboard.putNumber("Elevator Current: ", RobotMap.elevatorTalon1.getOutputCurrent());
		SmartDashboard.putNumber("Elevator Current 2: ", RobotMap.elevatorTalon1.getOutputCurrent());

		SmartDashboard.putNumber("Elevator Current Setpoint: ", RobotMap.elevatorTalon1.getClosedLoopTarget());
		SmartDashboard.putNumber("Elevator Current Error: ", RobotMap.elevatorTalon1.getClosedLoopError());

		SmartDashboard.putNumber("Elevator Velocity Setpoint: ", RobotMap.elevator.elevatorVelocityPID.getSetpoint());
		SmartDashboard.putNumber("Elevator Velocity Error: ", RobotMap.elevator.elevatorVelocityPID.getAvgError());
		SmartDashboard.putNumber("Elevator Velocity: ", RobotMap.elevatorEncoderWrapperRate.pidGet());
		SmartDashboard.putNumber("Elevator Velocity Get Error", RobotMap.elevator.elevatorVelocityPID.getError());

		SmartDashboard.putBoolean("Elevator Velocity Enabled:", RobotMap.elevator.elevatorVelocityPID.isEnabled());


		SmartDashboard.putNumber("Elevator Velocity Output: ", RobotMap.elevatorWrapperCurrent.get());
		SmartDashboard.putNumber("PWM Wrapper Value Elevator", RobotMap.elevatorWrapperPercentOutput.get());

		SmartDashboard.putBoolean("Elevator Vel Enabled: ", RobotMap.elevator.elevatorVelocityPID.isEnabled());

		SmartDashboard.putNumber("Elevator Dist Setpoint: ", RobotMap.elevator.distanceSetpointTN.pidGet());
		SmartDashboard.putNumber("Elevator Dist Output: ", RobotMap.elevator.distanceOutputTN.pidGet());
		SmartDashboard.putNumber("Elevator Dist Error: ", RobotMap.elevator.elevatorDistPID.getError());
		SmartDashboard.putBoolean("Elevator Dist Enabled: ", RobotMap.elevator.elevatorDistPID.isEnabled());
		SmartDashboard.putNumber("kV: ", ConstantsIO.kV_elevatorVelocity);

		SmartDashboard.putNumber("kV Term: ", WarlordsPIDController.getkVTermGlobal());
	}

	
}
