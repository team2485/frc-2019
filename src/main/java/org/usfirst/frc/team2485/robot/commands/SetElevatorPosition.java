package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetElevatorPosition extends InstantCommand {

    private double elevatorPosition;
    private ElevatorLevel elevatorLevel;
    private boolean manualMovement = false;
    private boolean encoderMovement = true;
    private long startEncoderLossTime = 0;
    private int fullPowerTime = 500;
    private boolean finish = false;

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
        if(!manualMovement) {
            RobotMap.elevator.setPosition(elevatorLevel.getPosition());
            ElevatorWithControllers.power = RobotMap.elevatorEncoderWrapperDistance.pidGet();
            if(RobotMap.elevator.distanceOutputTN.pidGet() > 5 && RobotMap.elevatorEncoderWrapperDistance.pidGet() == 0 && encoderMovement) {
                RobotMap.elevator.enablePID(false);
                encoderMovement = false;
                startEncoderLossTime = System.currentTimeMillis();
                manualMovement = true;
                ElevatorWithControllers.manualMovement = true;
            } if(!encoderMovement) {
                if(RobotMap.elevatorEncoderWrapperDistance.pidGet() != 0) {
                    encoderMovement = true;
                } 
                if(System.currentTimeMillis() - startEncoderLossTime >= fullPowerTime && elevatorLevel.getPosition() <= 6) {
                    manualMovement = true;
                }
            }
        } else {
            RobotMap.elevator.enablePID(false);
            RobotMap.elevator.setPosition(0);
            RobotMap.elevatorCurrent.set(0);
            RobotMap.elevatorPercentOutput.set(0);
            finish = true;
        }
    }

    @Override
    protected boolean isFinished() {
        return Math.abs(elevatorLevel.getPosition() - RobotMap.elevatorEncoderWrapperDistance.pidGet()) < 1 || finish;
    }

    @Override
    protected void end() {
        super.end();
    }

}