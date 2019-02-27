package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class Lift extends InstantCommand {
    boolean lift;

    public Lift(boolean lift) {
        requires(RobotMap.hatchIntake);
        this.lift = lift;
        setInterruptible(true);
    }

    @Override
    protected void initialize() {
        if(lift) {
            RobotMap.hatchIntake.lift();
        } else {
            RobotMap.hatchIntake.stow();
        }
    }
}