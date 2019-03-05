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
        setInterruptible(false);
    }
    protected void execute(){
        System.out.println("Driving straight");
        finished = RobotMap.driveTrain.distancePID.isOnTarget();
    }

    @Override
    protected boolean isFinished() {
        return finished || (System.currentTimeMillis() - startTime) > timeout;
    }

    @Override
	protected void end() {
        RobotMap.driveLeftEncoder.reset();
        RobotMap.driveRightEncoder.reset();
        super.end();
    }

}