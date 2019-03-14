package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.CargoArm;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.FastMath;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;

public class CargoArmWithControllers extends Command {
    private double threshold = 0.05;
    private double cargoArmSpikeCurrent = 5;
    private boolean spiking;
    private long startSpikeTime;
    private int spikeTime = 500;
    public static double power;
    public static boolean init = true;

    public CargoArmWithControllers() {
        setInterruptible(true);
        requires(RobotMap.cargoArm);
    }

    protected void initialize() {
       
    }

    @Override
    protected void execute() {
        System.out.println("Init: " + init);
        if(init) {
            RobotMap.cargoArm.enablePID(false);
            RobotMap.cargoArmCurrent.set(RobotMap.cargoArm.STARTING_CURRENT);
            if(RobotMap.cargoArmLimitSwitchUp.get()) {
                RobotMap.cargoArm.enablePID(true);
                RobotMap.cargoArmEncoder.reset();
                power = 0;
                init = false;
            }
        } else {
            boolean up = ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_RYJOYSTICK_PORT), 0.2, 0, 1) < 0;
            boolean zero = ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_RYJOYSTICK_PORT), 0.2, 0, 1) == 0;


            if(up && !zero) {
                power = -ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_RYJOYSTICK_PORT), 0.2, 0.2, -RobotMap.cargoArmEncoderWrapperDistance.pidGet());
            } else if (!zero) {
                power = -ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_RYJOYSTICK_PORT), 0.2, -RobotMap.cargoArmEncoderWrapperDistance.pidGet(), 1.9);
            } 
            System.out.println("Power: " + power);

            RobotMap.cargoArm.distanceSetpointTN.setOutput(power);

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
            System.out.println(System.currentTimeMillis() - startSpikeTime);

            if(RobotMap.cargoArmTalon.getOutputCurrent() >= cargoArmSpikeCurrent && !RobotMap.cargoArm.distancePID.isOnTarget()) {
                if(!spiking) {
                    startSpikeTime = System.currentTimeMillis();
                    spiking = true;
                }
            } else {
                spiking = false;
                RobotMap.cargoArm.failsafeTN.setOutput(0);
            } 
            if(spiking) {
                RobotMap.cargoArm.failsafeTN.setOutput(RobotMap.cargoArm.HOLDING_CURRENT);
            }
            if(RobotMap.cargoArmLimitSwitchUp.get() && power == 0) {
                RobotMap.cargoArm.failsafeTN.setOutput(RobotMap.cargoArm.HOLDING_CURRENT);
            } else {
                RobotMap.cargoArm.failsafeTN.setOutput(0);
            }
        }

    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}