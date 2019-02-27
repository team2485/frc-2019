package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class CargoRollersIntake extends Command {
    private double power;
    private double holdingCurrent, cargoIntakenCurrent;
    private long startSpikeTime, spikeTime;
    private boolean intakingMode, spiking;

    public CargoRollersIntake(double power) {
        requires(RobotMap.cargoRollers);
        requires(RobotMap.cargoArm);
        requires(RobotMap.hatchIntake);
        
        setInterruptible(true);
        this.power = power;
        holdingCurrent = 5;
        cargoIntakenCurrent = 10;
        spikeTime = 1000;
        intakingMode = true;
        spiking = false;
    } 

    @Override
    protected void initialize() {
        RobotMap.hatchIntake.slideIn();
        RobotMap.hatchIntake.stow();
        RobotMap.cargoArm.setPosition(0);
        RobotMap.cargoRollersPercentOutput.set(power);
    }

    @Override
    protected void execute() {
        if(intakingMode) {
            if(RobotMap.cargoRollersTalon.getOutputCurrent() >= cargoIntakenCurrent) {
                if(!spiking) {
                    startSpikeTime = System.currentTimeMillis();
                    spiking = true;
                }
            } else {
                spiking = false;
            } 
            if(spiking) {
                if(System.currentTimeMillis() - startSpikeTime > spikeTime) {
                    intakingMode = false;
                }
            }
        } else {
            RobotMap.cargoRollersCurrent.set(holdingCurrent);
            RobotMap.cargoArm.setPosition(1.8);
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        
    }
}