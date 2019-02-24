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
        double throttle = ThresholdHandler.deadbandAndScale(OI.jacket.getRawAxis(OI.XBOX_RTRIGGER_PORT), 0.2, 0, 1) 
                        - ThresholdHandler.deadbandAndScale(OI.jacket.getRawAxis(OI.XBOX_LTRIGGER_PORT), 0.2, 0, 1);
        double steering = ThresholdHandler.deadbandAndScale(OI.jacket.getRawAxis(OI.XBOX_LXJOSYSTICK_PORT), 0.2, 0, 1);
        boolean quickTurn = OI.jacket.getRawButton(OI.XBOX_X_PORT);

        RobotMap.driveTrain.WarlordsDrive(throttle, steering, quickTurn);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}