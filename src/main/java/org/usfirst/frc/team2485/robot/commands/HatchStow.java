package org.usfirst.frc.team2485.robot.commands;

import java.lang.module.ModuleDescriptor.Requires;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class HatchStow extends InstantCommand {

    public HatchStow(){
        requires(RobotMap.hatchIntake);
        setInterruptible(true);
    }

    protected void initialize(){
        RobotMap.hatchIntake.stow(); 
    }
}