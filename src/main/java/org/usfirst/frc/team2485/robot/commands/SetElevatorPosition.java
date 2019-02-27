package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;

import edu.wpi.first.wpilibj.command.Command;

public class SetElevatorPosition extends Command {
    private double elevatorPosition;
    private ElevatorLevel elevatorLevel;

    public SetElevatorPosition(ElevatorLevel elevatorLevel) {
        requires(RobotMap.elevator);
        setInterruptible(true);
        this.elevatorPosition = elevatorLevel.getPosition();
        this.elevatorLevel = elevatorLevel;
    }

    public SetElevatorPosition(double elevatorPosition) {
        requires(RobotMap.elevator);
        setInterruptible(true);
        this.elevatorPosition = elevatorPosition;
    }

    @Override
    protected void initialize() {
        System.out.println("Elevator Output");
        RobotMap.elevator.distanceSetpointTN.setOutput(elevatorPosition);
        RobotMap.elevator.distancePID.setAbsoluteTolerance(1);
        RobotMap.elevator.enablePID(true);
        if(elevatorLevel != null) {
            RobotMap.elevator.lastLevel = elevatorLevel;
            RobotMap.elevator.setElevatorPosition(elevatorLevel);
        }
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