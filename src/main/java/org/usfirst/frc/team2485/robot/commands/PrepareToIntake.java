package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class PrepareToIntake extends InstantCommand {
    public PrepareToIntake() {
        requires(RobotMap.hatchIntake);
        requires(RobotMap.hatchRollers);
    }

    @Override
    protected void initialize() {
        RobotMap.hatchIntake.lift();
        RobotMap.hatchIntake.slideOut();
        // RobotMap.hatchRollers.hatchRollersManual(0);
        RobotMap.compressor.setClosedLoopControl(false);
    }
}