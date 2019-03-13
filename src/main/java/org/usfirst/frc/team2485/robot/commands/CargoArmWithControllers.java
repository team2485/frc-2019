package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.CargoArm;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.FastMath;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;

public class CargoArmWithControllers extends Command {
    private static final double CURRENT_THRESHOLD = 15;
    private double threshold = 0.05;
    private double cargoArmSpikeCurrent = 10;
    private boolean spiking;
    private long startSpikeTime;
    private int spikeTime = 1000;

    public CargoArmWithControllers() {
        setInterruptible(true);
        requires(RobotMap.cargoArm);
    }

    protected void initialze(){
        RobotMap.cargoArm.enablePID(false);
        RobotMap.cargoArmCurrent.set(RobotMap.cargoArm.HOLDING_CURRENT);
        while(RobotMap.cargoArmLimitSwitchUp.get()) {
            RobotMap.cargoArm.distancePID.enable();
        }
    }

    @Override
    protected void execute() {
        boolean up = ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_RYJOYSTICK_PORT), 0.2, 0, 1) > 0;
        boolean zero = ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_RYJOYSTICK_PORT), 0.2, 0, 1) == 0;
        double power = 0;

        if(up) {
            power = ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_RYJOYSTICK_PORT), 0.2, RobotMap.cargoArmEncoderWrapperDistance.pidGet(), 0);
        } else if (!up && !zero) {
            power = ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_RYJOYSTICK_PORT), 0.2, -1.7, RobotMap.cargoArmEncoderWrapperDistance.pidGet());
        }

        if(RobotMap.cargoArmLimitSwitchUp.get()) {
            RobotMap.cargoArm.distancePID.resetIntegrator();
        }

        if(Math.abs(RobotMap.cargoArmEncoderWrapperDistance.pidGet() + 1.7) < threshold) {
            RobotMap.cargoArm.distanceRampRate.setRampRates(ConstantsIO.armDistanceSetpointUpRamp, ConstantsIO.armDistanceSetpointDownRampClose);
        } else if(Math.abs(RobotMap.cargoArmEncoderWrapperDistance.pidGet() - 0) < threshold) {
            RobotMap.cargoArm.distanceRampRate.setRampRates(ConstantsIO.armDistanceSetpointUpRampClose, ConstantsIO.armDistanceSetpointDownRamp);
        } else {
            RobotMap.cargoArm.distanceRampRate.setRampRates(ConstantsIO.armDistanceSetpointUpRamp, ConstantsIO.armDistanceSetpointDownRamp);

        }


        if(RobotMap.cargoArmTalon.getOutputCurrent() >= cargoArmSpikeCurrent) {
            if(!spiking) {
                startSpikeTime = System.currentTimeMillis();
                spiking = true;
            }
        } else {
            spiking = false;
            RobotMap.cargoArm.failsafeTN.setOutput(0);
        } 
        if(spiking) {
            if(System.currentTimeMillis() - startSpikeTime > spikeTime) {
                RobotMap.cargoArm.failsafeTN.setOutput(RobotMap.cargoArm.HOLDING_CURRENT);
            }
        }

    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}