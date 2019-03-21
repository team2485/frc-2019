package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class Hook extends InstantCommand {
    boolean hook;

    public Hook(boolean hook) {
        requires(RobotMap.hatchIntake);
        this.hook = hook;
        setInterruptible(true);
    }

    @Override
    protected void initialize() {
        RobotMap.warlordsCompressor.airRemaining -= 10;
        // if(hook) {
        //     RobotMap.hatchIntake.hookOut();
        // } else {
        //     RobotMap.hatchIntake.hookIn();
        // }
    }
}