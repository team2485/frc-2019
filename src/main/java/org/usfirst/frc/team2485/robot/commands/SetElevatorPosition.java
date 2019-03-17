package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetElevatorPosition extends InstantCommand {

    private double elevatorPosition;
    private ElevatorLevel elevatorLevel;

    public SetElevatorPosition(ElevatorLevel elevatorLevel) {
        requires(RobotMap.elevator);
        setInterruptible(true);
        this.elevatorPosition = elevatorLevel.getPosition();
        this.elevatorLevel = elevatorLevel;
    }

    @Override
    protected void initialize() {
        RobotMap.elevator.lastLevel = elevatorLevel;
        RobotMap.elevator.setPosition(elevatorLevel.getPosition());
        RobotMap.elevator.enablePID(true);
    }

    @Override
    protected void execute() {
        RobotMap.elevator.setPosition(elevatorLevel.getPosition());
        ElevatorWithControllers.power = RobotMap.elevatorEncoderWrapperDistance.pidGet();
    }

    @Override
    protected boolean isFinished() {
        return Math.abs(elevatorLevel.getPosition() - RobotMap.elevatorEncoderWrapperDistance.pidGet()) < 1;
    }

    @Override
    protected void end() {
        super.end();
    }

}