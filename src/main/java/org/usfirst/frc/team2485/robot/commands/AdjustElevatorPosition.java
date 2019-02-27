package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;

import edu.wpi.first.wpilibj.command.Command;

public class AdjustElevatorPosition extends Command {

    private double adjust;

    public AdjustElevatorPosition(double adjust) {
        requires(RobotMap.elevator);
        setInterruptible(true);
        this.adjust = adjust;
    }
    @Override
    protected void initialize() {
        RobotMap.elevator.setPosition(RobotMap.elevatorEncoderWrapperDistance.pidGet() + adjust);
    }

    @Override
    protected void execute() {
        
    }

    @Override
    protected boolean isFinished() {
        return RobotMap.elevator.distancePID.isOnTarget();
    }

    @Override
    protected void end() {
        super.end();
    }

}