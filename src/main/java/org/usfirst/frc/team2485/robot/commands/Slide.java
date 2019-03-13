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
        setInterruptible(true);
    }

    @Override
    protected void initialize() {
        RobotMap.warlordsCompressor.airRemaining -= 10;
        if(slide) {
            RobotMap.hatchIntake.slideOut();
        } else {
            RobotMap.hatchIntake.slideIn();
        }
    }
}