package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetHatchRollersCurrent extends InstantCommand {
    private double curr;

    public SetHatchRollersCurrent(double curr) {
        requires(RobotMap.hatchRollers);
        this.curr = curr;
    }

    @Override
    protected void initialize() {
        RobotMap.hatchRollersCurrent.set(curr);
    }
}