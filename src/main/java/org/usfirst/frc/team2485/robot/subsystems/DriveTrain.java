package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;

import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveTrain extends Subsystem {

    public DriveTrain() {

    }

    public void WarlordsDrive(double throttle, double steering, boolean quickTurn) {
        if(quickTurn) {
            RobotMap.driveLeftPercentOutput.set(steering);
            RobotMap.driveRightPercentOutput.set(-steering);
        } else {
            double left = throttle + Math.abs(throttle)*steering;
            double right = throttle - Math.abs(throttle)*steering;

            if(Math.abs(left) > 1) {
                right /= Math.abs(left);
                left /= Math.abs(left);
            } else if (Math.abs(right) > 1) {
                left /= Math.abs(right);
                right /= Math.abs(right);
            }

            RobotMap.driveLeftPercentOutput.set(left);
            RobotMap.driveRightPercentOutput.set(right);
        }
    }

    public void initDefaultCommand() {
        setDefaultCommand(new DriveWithControllers());
    }

    public void updateConstants() {
        
    }
}