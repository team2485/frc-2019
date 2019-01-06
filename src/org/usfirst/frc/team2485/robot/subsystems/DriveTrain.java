package org.usfirst.frc.team2485.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Drivetrain extends Subsystem {

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new DriveWithControllers());
    }
    
    public void simpleDrive(double x, double y, boolean quickTurn) {
    	
    	double steering = ThresholdHandler.deadbandAndScale(x, 0.2, 0, 1);
		
		double throttle = ThresholdHandler.deadbandAndScale(y, 0.2, 0, 1);
		
		double left;
		
		double right;
		
		if (quickTurn) {
			left=steering;
			right=-steering;
		} else {
			left=throttle+steering*throttle;
			right=throttle-steering*throttle;
			if (Math.abs(left)>1) {
				right/=Math.abs(left);
				left/=Math.abs(left);
			}
			
			if (Math.abs(right)>1) {
				right/=Math.abs(right);
				left/=Math.abs(right);
			}
		}
		
		
		RobotMap.driveLeft.set(left);
		
		RobotMap.driveRight.set(right);
		
    }
    
    public void driveAtSpeed(double speed) {
    	RobotMap.driveLeft.set(speed);
    	RobotMap.driveRight.set(speed);
    }
    
    public void stop() {
    	RobotMap.driveLeft.set(0);
    	RobotMap.driveRight.set(0);
    }
}

