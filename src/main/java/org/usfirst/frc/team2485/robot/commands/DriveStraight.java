package org.usfirst.frc.team2485.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2485.robot.RobotMap;

public class DriveStraight extends Command {

    private double dist;
	private boolean finished;
	private int timeout;
    private long startTime, startSpikeTime;
    private int driveTrainStallingCurrent;
    private double spikeTime;
    private boolean moving, spiking;
    
    public DriveStraight(double dist, int timeout){
        this.dist = dist;
        this.timeout = timeout;
        requires(RobotMap.driveTrain);
        setInterruptible(false);
    }

     @Override
    protected void initialize() {
        super.initialize();
        driveTrainStallingCurrent = 15;
        spikeTime = 1000;
        moving = true;
        spiking = false;
        startTime = System.currentTimeMillis();
        RobotMap.driveTrain.distancePID.enable();
        RobotMap.driveTrain.velocityPID.enable();
        RobotMap.driveTrain.anglePID.enable();
        RobotMap.driveTrain.angVelPID.enable();
        RobotMap.driveTrain.leftMotorSetter.enable();
        RobotMap.driveTrain.rightMotorSetter.enable();
        RobotMap.driveTrain.distanceSetpointTN.setOutput(dist);
        RobotMap.driveTrain.angleSetpointTN.setOutput(0);
    }
    protected void execute(){
        System.out.println("Driving straight");
        if(moving){
            if((RobotMap.driveLeftTalon1.getOutputCurrent() >= driveTrainStallingCurrent) && (RobotMap.driveRightTalon1.getOutputCurrent() >= driveTrainStallingCurrent)) { //we should check both sides just because...                if(!spiking) {
                    startSpikeTime = System.currentTimeMillis();
                    spiking = true;
                }
            } else {
                spiking = false;
        }

        if(spiking) {
            if(System.currentTimeMillis() - startSpikeTime > spikeTime) {
                moving = false;
                RobotMap.driveTrain.distanceSetpointTN.setOutput(0); 
                }
        }    
        finished = RobotMap.driveTrain.distancePID.isOnTarget();
    }


    @Override
    protected boolean isFinished() {
        // return (System.currentTimeMillis() - startTime) > timeout;
        System.out.println("Running");
        if(finished || (spiking && !moving)){
            RobotMap.driveLeftEncoder.reset();
            RobotMap.driveRightEncoder.reset();
            return true;
        } else {
            return false;
        }
       

    }

    @Override
	protected void end() {
        System.out.println("Ended");
    //    RobotMap.driveTrain.enablePID(false);
        super.end();
    }

}