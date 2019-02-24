package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class Slide extends InstantCommand {
    boolean slide;

    public Slide(boolean slide) {
        requires(RobotMap.hatchIntake);
        this.slide = slide;
    }

    @Override
    protected void initialize() {
        if(slide) {
            RobotMap.hatchIntake.lift();
        } else {
            RobotMap.hatchIntake.stow();
        }
    }
}