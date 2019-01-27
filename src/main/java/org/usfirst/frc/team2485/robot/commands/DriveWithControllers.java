package org.usfirst.frc.team2485.robot.commands;
import edu.wpi.first.wpilibj.command.Command;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import org.usfirst.frc.team2485.robot.subsystems.DriveTrain;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class DriveWithControllers extends Command {

    public DriveWithControllers() {
        setInterruptible(true);
        requires(RobotMap.driveTrain);
    }

    // Called just before this Command runs the first time
    protected void initialize() {
        RobotMap.driveTrain.enablePID(false);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
       
        double forwardThrottleUA=OI.xbox.getRawAxis(3);
        double backThrottleUA=OI.xbox.getRawAxis(2);
    	double steeringUA=OI.xbox.getRawAxis(0);
        boolean quickTurn=OI.xbox.getRawButton(3);
        // System.out.println("Forward - Backward: " + (forwardThrottleUA - backThrottleUA));
        // System.out.println("SteeringUA: " + steeringUA);
    	RobotMap.driveTrain.simpleDrive(forwardThrottleUA-backThrottleUA,steeringUA,quickTurn);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
