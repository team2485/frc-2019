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

import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;
import org.usfirst.frc.team2485.robot.subsystems.DriveTrain;
import  org.usfirst.frc.team2485.util.ConstantsIO;
import  org.usfirst.frc.team2485.util.Pipeline;
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


	private static final AutoPath path = new AutoPath (AutoPath.getPointsForBezier(2000, new Pair(0.0, 0.0), new Pair(0, 44.0),
		new Pair(53.5 - 6, 30.0), new Pair(53.5 - 6, 94)));


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
		RobotMap.init();
		OI.init();
		// ConstantsIO.init();
		RobotMap.driveTrain.updateConstants();

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
		RobotMap.driveTrain.enablePID(false);
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
		RobotMap.driveLeftEncoder.reset();
		RobotMap.driveRightEncoder.reset();
		RobotMap.gyroRateWrapper.reset();
		RobotMap.gyroAngleWrapper.reset();
		RobotMap.driveTrain.updateConstants();
		m_autonomousCommand = m_chooser.getSelected();
		ConstantsIO.init();
		RobotMap.driveTrain.enablePID(true);
		// Pair[] controlPoints = { new Pair( 35.6, -255.0), new Pair(0, -110.0), new Pair(0.0, 0.0) };
		
		// double[] dists = { 90.0, };
	   	// AutoPath path = AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);
		Scheduler.getInstance().add(new DriveTo(path, 50, false, 20000, false, false));
		
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
		
		// RobotMap.driveTrain.distancePID.setSetpoint(RobotMap.lidar.getDistance() - 20);

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
		updateSmartDashboard();
		RobotMap.driveTrain.updateConstants();
		//RobotMap.driveTrain.setAngle(Math.PI/2);
		

		
		// RobotMap.driveTrain.setHighLowCurrent(-0.5, -0.5, -0.5, -0.5, 1000);
		
	}

	@Override
	public void teleopInit() {
		RobotMap.driveTrain.updateConstants();
		RobotMap.driveTrain.enablePID(false);

		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
		Scheduler.getInstance().add(new DriveWithControllers());

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

		SmartDashboard.putNumber("Angle Output", RobotMap.driveTrain.angleTN.pidGet());

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

		SmartDashboard.putNumber("Ang Vel Setpoint", RobotMap.angVelPID.pidGet());
	
		SmartDashboard.putNumber("Lidar Value", RobotMap.lidar.getDistance());
	}

	
}
