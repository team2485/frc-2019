package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class SetAngle extends Command {
    private double angle;
    private double tolerance;
    public SetAngle(double angle, double tolerance) {
        this.angle = angle;
        this.tolerance = tolerance;
        requires(RobotMap.driveTrain);
        setInterruptible(false);
    }

    @Override
    protected boolean isFinished() {
        return RobotMap.driveTrain.setAngle(angle, tolerance);
    }


}