package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;

import edu.wpi.first.wpilibj.command.Command;

public class SetElevatorPosition extends Command {
    private ElevatorLevel elevatorLevel;

    public SetElevatorPosition(ElevatorLevel elevatorLevel) {
        requires(RobotMap.elevator);
        setInterruptible(true);
        this.elevatorLevel = elevatorLevel;
    }

    @Override
    protected void initialize() {
        System.out.println("Elevator Output");
        RobotMap.elevator.lastLevel = elevatorLevel;
        RobotMap.elevator.distanceSetpointTN.setOutput(elevatorLevel.getPosition());
        RobotMap.elevator.distancePID.setAbsoluteTolerance(1);
        RobotMap.elevator.enablePID(true);
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