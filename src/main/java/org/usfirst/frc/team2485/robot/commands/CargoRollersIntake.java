package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.CargoRollers;

import edu.wpi.first.wpilibj.command.Command;

public class CargoRollersIntake extends Command {
    private double power;
    private double holdingCurrent, cargoIntakenCurrent;
    private long startSpikeTime, spikeTime;
    private boolean intakingMode, spiking;

    public CargoRollersIntake(double power) {
        requires(RobotMap.cargoRollers);

        setInterruptible(true);
        this.power = power;
    } 

    @Override
    protected void initialize() {
        RobotMap.cargoRollersPercentOutput.set(power);
        holdingCurrent = CargoRollers.HOLDING_CURRENT;
        cargoIntakenCurrent = 12;
        spikeTime = 500;
        intakingMode = true;
        spiking = false;
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
            RobotMap.cargoRollersPercentOutput.set(0.3);
        }
    }

    @Override
    protected boolean isFinished() {
        
        return !intakingMode;
    }

    @Override
    protected void end() {
        RobotMap.cargoRollersPercentOutput.set(0.2);
    }
}