package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class EjectCargo extends Command {
    private long startTime;
    private int timeout;
    private double power;

    public EjectCargo(double power) {
        requires(RobotMap.cargoRollers);
        setInterruptible(true);
        this.power = power; 
        timeout = 1000;
    }

    @Override
    protected void initialize() {
        startTime = System.currentTimeMillis();
        RobotMap.cargoRollers.cargoRollersManual(power);
    }

    @Override
    protected boolean isFinished() {
        System.out.println(System.currentTimeMillis() - startTime);
        return System.currentTimeMillis() - startTime > timeout;
    }

    @Override
    protected void end() {
        RobotMap.cargoRollers.cargoRollersManual(0);
    }
}