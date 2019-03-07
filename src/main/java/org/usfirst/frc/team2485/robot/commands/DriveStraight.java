package org.usfirst.frc.team2485.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2485.robot.RobotMap;

public class DriveStraight extends Command {

    private double dist;
	private boolean finished;
	private int timeout;
	private long startTime;
    
    public DriveStraight(double dist, int timeout){
        this.dist = dist;
        this.timeout = timeout;
        requires(RobotMap.driveTrain);
        setInterruptible(false);
    }

     @Override
    protected void initialize() {
        super.initialize();
        startTime = System.currentTimeMillis();
        RobotMap.driveTrain.distancePID.enable();
        RobotMap.driveTrain.velocityPID.enable();
        RobotMap.driveTrain.leftMotorSetter.enable();
        RobotMap.driveTrain.rightMotorSetter.enable();
        RobotMap.driveTrain.distanceSetpointTN.setOutput(dist);
        // RobotMap.driveTrain.angleSetpointTN.setOutput(0);
    }
    protected void execute(){
        System.out.println("Driving straight");
        finished = RobotMap.driveTrain.distancePID.isOnTarget();
    }

    @Override
    protected boolean isFinished() {
        // return (System.currentTimeMillis() - startTime) > timeout;
        System.out.println("Running");
        return true;

    }

    @Override
	protected void end() {
        System.out.println("Ended");
    //    RobotMap.driveTrain.enablePID(false);
        super.end();
    }

}