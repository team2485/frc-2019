package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class Hook extends InstantCommand {
    boolean hook;

    public Hook(boolean hook) {
        requires(RobotMap.hatchIntake);
        this.hook = hook;
    }

    @Override
    protected void initialize() {
        if(hook) {
            RobotMap.hatchIntake.hookOut();
        } else {
            RobotMap.hatchIntake.hookIn();
        }
    }
}