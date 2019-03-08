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
        double steering = OI.getDriveSteering() * Math.abs(OI.getDriveSteering());
        boolean quickTurn = OI.getQuickTurn();

        // if(throttle == 0 && steering == 0) {
        //     throttle = ThresholdHandler.deadbandAndScale(OI.jacketBackup.getRawAxis(3) - OI.jacketBackup.getRawAxis(2), 0.1, 0, 1);
        //     steering = ThresholdHandler.deadbandAndScale(OI.jacketBackup.getRawAxis(0), 0.2, 0, 1);
        //     quickTurn = OI.jacketBackup.getRawButton(OI.XBOX_X_PORT);
        // }

        // if (quickTurn) {
        //     steering *= 0.7;
        // }

        RobotMap.driveTrain.WarlordsDrive(throttle, steering, quickTurn);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}