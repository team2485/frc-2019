package org.usfirst.frc.team2485.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team2485.robot.RobotMap;

public class SetAngle extends Command {

    private double angle;
	private boolean finished;
	private int timeout;
	private long startTime;
    
    public SetAngle(double angle, int timeout) {
        this.angle = angle;
        this.timeout = timeout;
        requires(RobotMap.driveTrain);
        setInterruptible(false);

    }

     @Override
    protected void initialize() {
        super.initialize();
        startTime = System.currentTimeMillis();
        RobotMap.driveTrain.anglePID.enable();
        RobotMap.driveTrain.angVelPID.enable();
        RobotMap.driveTrain.leftMotorSetter.enable();
        RobotMap.driveTrain.rightMotorSetter.enable();
        RobotMap.driveTrain.angleSetpointTN.setOutput(angle);
    }
    protected void execute(){
        // System.out.println("Angling");
        finished = RobotMap.driveTrain.anglePID.isOnTarget();
    }

    @Override
    protected boolean isFinished() {
        return finished || (System.currentTimeMillis() - startTime) > timeout;
        // return false;
    }

    @Override
	protected void end() {
        // RobotMap.driveLeftEncoder.reset();
        // RobotMap.driveRightEncoder.reset();
        super.end();
    }

}