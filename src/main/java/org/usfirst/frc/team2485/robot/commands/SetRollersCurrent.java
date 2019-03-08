package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetRollersCurrent extends InstantCommand {
    public SetRollersCurrent() {
        requires(RobotMap.cargoRollers);
    }

    @Override
    protected void initialize() {
        RobotMap.cargoRollersCurrent.set(RobotMap.cargoRollers.HOLDING_CURRENT);
    }
}