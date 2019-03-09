package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.CargoArm;

import edu.wpi.first.wpilibj.command.Command;

public class HoldingCurrent extends Command {

    public HoldingCurrent() {
        requires(RobotMap.cargoArm);
        setInterruptible(true);
    }

    protected void initialize() {
        RobotMap.cargoArm.enablePID(false);
    }

    protected void execute() {
        RobotMap.cargoArmTalonWrapperCurrent.set(CargoArm.HOLDING_CURRENT);
        RobotMap.cargoArm.holdPosition = 0;
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}