package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;

import edu.wpi.first.wpilibj.command.Command;

public class SetElevatorPosition extends Command {

    private double elevatorPosition;
    private ElevatorLevel elevatorLevel;

    public SetElevatorPosition(ElevatorLevel elevatorLevel) {
        requires(RobotMap.elevator);
        setInterruptible(false);
        this.elevatorPosition = elevatorLevel.getPosition();
        this.elevatorLevel = elevatorLevel;
    }

    @Override
    protected void initialize() {
        RobotMap.elevator.setPosition(elevatorLevel.getPosition());
    }

    @Override
    protected void execute() {
        RobotMap.elevator.setPosition(elevatorLevel.getPosition());
        System.out.println("Elevator Pos: " + elevatorLevel.getPosition());
        ElevatorWithControllers.holdPosition = RobotMap.elevatorEncoderWrapperDistance.pidGet();
        System.out.println("Hold Position: " + ElevatorWithControllers.holdPosition);
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