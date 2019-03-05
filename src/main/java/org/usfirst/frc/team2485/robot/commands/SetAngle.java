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
    }

     @Override
    protected void initialize() {
        super.initialize();
        startTime = System.currentTimeMillis();
        RobotMap.driveTrain.anglePID.setSetpoint(angle);
        setInterruptible(false);
    }
    protected void execute(){
        finished = RobotMap.driveTrain.anglePID.isOnTarget();
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