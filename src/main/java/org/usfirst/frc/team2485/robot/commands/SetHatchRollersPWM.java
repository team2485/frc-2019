package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetHatchRollersPWM extends Command {
    private double power;

    public SetHatchRollersPWM(double power) {
        requires(RobotMap.hatchRollers);
        setInterruptible(true);
        this.power = power;
    }

    @Override
    protected void initialize() {
        RobotMap.hatchRollers.hatchRollersManual(power);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}