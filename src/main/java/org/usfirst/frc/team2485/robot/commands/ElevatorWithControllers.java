package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.Elevator;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorWithControllers extends Command {
    public static double holdPosition = 0; 
    private double elevatorSpikeCurrent = 10;
    private boolean spiking;
    private long startSpikeTime;
    private int spikeTime = 20000;
    public static double power = 0;

    public ElevatorWithControllers() {
        setInterruptible(true);
        requires(RobotMap.elevator);
    }

    @Override
    protected void initialize() {
        holdPosition = RobotMap.elevator.lastLevel.getPosition();
       
        RobotMap.elevator.distanceSetpointTN.setOutput(power); //so that handoff between SetElevatorPosition is smooth :)

        RobotMap.elevator.enablePID(true);
    }

    @Override
    protected void execute() {
        // dont make same ramp rate changes
        boolean up = -ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_LYJOYSTICK_PORT), 0.2, 0, 1) > 0;
        boolean zero = ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_LYJOYSTICK_PORT), 0.2, 0, 1) == 0;

        if(up) {
            power = -ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_LYJOYSTICK_PORT), 0.2, RobotMap.elevatorEncoderWrapperDistance.pidGet(), ElevatorLevel.ROCKET_LEVEL_THREE.getPosition());
        } else if (!up && !zero) {
            power = ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_LYJOYSTICK_PORT), 0.2, RobotMap.elevatorEncoderWrapperDistance.pidGet(), 0);
        }

        RobotMap.elevator.distanceSetpointTN.setOutput(power);

        if(RobotMap.elevatorEncoder.pidGet() <= 0) {
            RobotMap.elevator.distancePID.resetIntegrator();
        }



        if(RobotMap.elevatorTalon1.getOutputCurrent() >= elevatorSpikeCurrent && !RobotMap.elevator.distancePID.isOnTarget()) {
            if(!spiking) {
                startSpikeTime = System.currentTimeMillis();
                spiking = true;
            }
        } else {
            spiking = false;
            RobotMap.elevator.enableFailsafe = false;
        } 
        if(spiking && System.currentTimeMillis() - startSpikeTime >= spikeTime) {   
                RobotMap.elevator.enableFailsafe = true; //make sure this value can actually hold elevator up :)
        }



       

    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}