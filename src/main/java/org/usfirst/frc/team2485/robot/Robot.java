/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2485.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2485.robot.commandGroups.LoadingStationIntake;
import org.usfirst.frc.team2485.robot.commands.EjectCargo;
import org.usfirst.frc.team2485.robot.commands.SetArmPosition;
import org.usfirst.frc.team2485.robot.commands.SetElevatorPosition;
import org.usfirst.frc.team2485.robot.commands.SetRollers;
import org.usfirst.frc.team2485.robot.subsystems.CargoArm;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.FastMath;





/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */

public class Robot extends TimedRobot {
	
	boolean ejecting = false;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		
		FastMath.init();
		// chooser.addObject("My Auto", new MyAutoCommand());
		ConstantsIO.init();
		RobotMap.init();
		OI.init();

	
	
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
		RobotMap.updateConstants();
	

		//RobotMap.driveTrain.enablePID(true);
		// Pair[] controlPoints = { new Pair( 35.6, -255.0), new Pair(0, -110.0), new Pair(0.0, 0.0) };
		
		// double[] dists = { 90.0, };
	   	// AutoPath path = AutoPath.getAutoPathForClothoidSpline(controlPoints, dists);
		// Scheduler.getInstance().add(new DriveTo(path, 50, false, 20000, false, false));
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

		RobotMap.elevator.enablePID(true);
		// RobotMap.elevator.setElevatorPosition(30);


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
		ConstantsIO.init();
		RobotMap.cargoArmEncoder.reset();
		RobotMap.elevatorEncoder.reset();
		RobotMap.updateConstants();
		RobotMap.hatchIntake.slideIn();
		RobotMap.hatchIntake.hookIn();
		RobotMap.hatchIntake.retractPushers();
		RobotMap.hatchIntake.stow();
		Scheduler.getInstance().add(new SetRollers(0));
		// Scheduler.getInstance().add(new SetArmPosition(CargoArm.TOP_POSITION));
		
		UsbCamera jevoisCam = CameraServer.getInstance().startAutomaticCapture();
		jevoisCam.setVideoMode(PixelFormat.kYUYV, 160, 120, 30);

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


		if (RobotMap.cargoArmLimitSwitchDown.get()) {
			RobotMap.cargoArmEncoder.reset();
		} else if (RobotMap.cargoArmLimitSwitchUp.get()) {
			
		}


	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		updateSmartDashboard();
	}

	public void updateSmartDashboard() {
		SmartDashboard.putNumber("Elevator Distance PID Setpoint: ", RobotMap.elevator.distancePID.getSetpoint());
		SmartDashboard.putNumber("Elevator Distance PID Error: ", RobotMap.elevator.distancePID.getError());
		SmartDashboard.putNumber("Elevator Setpoint TN: ", RobotMap.elevator.distanceSetpointTN.getOutput());
		SmartDashboard.putNumber("Elevator Setpoint Ramped TN: ", RobotMap.elevator.distanceSetpointRampedTN.getOutput());
		SmartDashboard.putBoolean("Elevator Setpoint Ramp Rate Enabled: ", RobotMap.elevator.distanceSetpointRampRate.isEnabled());
		SmartDashboard.putBoolean("Elevator Distance PID Enabled: ", RobotMap.elevator.distancePID.isEnabled());
		SmartDashboard.putNumber("Elevator Position:", RobotMap.elevatorEncoderWrapperDistance.pidGet());
		SmartDashboard.putNumber("Elevator Hatch Placement: ", RobotMap.elevatorEncoderWrapperDistance.pidGet() - 3);
		SmartDashboard.putNumber("Elevator Output Current: ", RobotMap.elevatorTalon1.getOutputCurrent());


		SmartDashboard.putNumber("Cargo Arm Distance PID Setpoint: ", RobotMap.cargoArm.distancePID.getSetpoint());
		SmartDashboard.putNumber("Cargo Arm Distance PID Error: ", RobotMap.cargoArm.distancePID.getError());
		SmartDashboard.putNumber("Cargo Arm Setpoint TN: ", RobotMap.cargoArm.distanceSetpointTN.getOutput());
		SmartDashboard.putBoolean("Cargo Arm Distance PID Enabled: ", RobotMap.cargoArm.distancePID.isEnabled());
		SmartDashboard.putNumber("Cargo Arm Position: ", RobotMap.cargoArmEncoderWrapperDistance.pidGet());	
		SmartDashboard.putNumber("Cargo Arm PID Source Output: ", RobotMap.cargoArm.distanceOutputPIDSource.pidGet());
		SmartDashboard.putBoolean("Cargo Arm on position:", RobotMap.cargoArm.distancePID.isOnTarget());
		SmartDashboard.putBoolean("Cargo Arm Up Limit Switch: ", RobotMap.cargoArmLimitSwitchUp.get());
		SmartDashboard.putBoolean("Cargo Arm Down Limit Switch: ", RobotMap.cargoArmLimitSwitchDown.get());
		SmartDashboard.putNumber("Cargo Arm Output Current: ", RobotMap.cargoArmTalon.getOutputCurrent());
		SmartDashboard.putNumber("Cargo Roller Current", RobotMap.cargoRollersTalon.getOutputCurrent());

	}

	
}
