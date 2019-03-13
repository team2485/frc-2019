package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class Pushers extends InstantCommand {
    boolean pushers;

    public Pushers(boolean pushers) {
        requires(RobotMap.hatchIntake);
        this.pushers = pushers;
    }

    @Override
    protected void initialize() {
        RobotMap.warlordsCompressor.airRemaining -= 10;
        if(pushers) {
            RobotMap.hatchIntake.extendPushers();
        } else {
            RobotMap.hatchIntake.retractPushers();
        }
    }
}