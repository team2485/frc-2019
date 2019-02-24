package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetRollers extends InstantCommand {
    private double power;

    public SetRollers(double power) {
        requires(RobotMap.cargoRollers);
        this.power = power;
    }

    @Override
    protected void initialize() {
        RobotMap.cargoRollers.cargoRollersManual(power);
    }
}