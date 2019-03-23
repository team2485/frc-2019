package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class HatchRollersIntake extends Command {

    private double power;
    private double holdingCurrent, hatchIntakenCurrent;
    private long startSpikeTime, spikeTime;
    private boolean intakingMode, spiking;

    public HatchRollersIntake(double power) {
        requires(RobotMap.hatchRollers);

        setInterruptible(true);
        this.power = power;
    } 

    @Override
    protected void initialize() {
        RobotMap.hatchRollers.hatchRollersManual(power);
       // holdingCurrent = CargoRollers.HOLDING_CURRENT;
        hatchIntakenCurrent = 35; //spikes at 60 amps????
        spikeTime = 1000;
        intakingMode = true;
        spiking = false;
    }

    @Override
    protected void execute() {
        if(intakingMode) {
            if(RobotMap.hatchRollersTalon.getOutputCurrent() >= hatchIntakenCurrent) {
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
                    RobotMap.hatchRollers.intaken = true;
                }
            }
        } else {

            RobotMap.hatchRollers.hatchRollersManual(0.1);
        }
    }

    @Override
    protected boolean isFinished() {
        
        return !intakingMode;
    }

    @Override
    protected void end() {
        RobotMap.hatchRollers.hatchRollersManual(0.1);
    }

   


}