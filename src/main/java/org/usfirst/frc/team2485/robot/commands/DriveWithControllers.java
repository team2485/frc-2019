package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;

public class DriveWithControllers extends Command {
    public DriveWithControllers() {
        setInterruptible(true);
        requires(RobotMap.driveTrain);
    }

    @Override
    protected void execute() {
        double throttle = OI.getDriveThrottle();
        double steering = OI.getDriveSteering();
        boolean quickTurn = OI.getQuickTurn();

        if (quickTurn) {
            steering *= 0.7;
        }

        RobotMap.driveTrain.WarlordsDrive(throttle, steering, quickTurn);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}