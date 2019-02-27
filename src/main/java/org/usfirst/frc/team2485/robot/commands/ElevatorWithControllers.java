package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.Elevator;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorWithControllers extends Command {
    double holdPosition;

    public ElevatorWithControllers() {
        setInterruptible(true);
        requires(RobotMap.elevator);
    }

    @Override
    protected void initialize() {
        holdPosition = RobotMap.elevator.lastLevel.getPosition();
    }

    @Override
    protected void execute() {
        double power = ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_LYJOYSTICK_PORT), 0.2, 0, 0.6);

        if(power != 0) {
            RobotMap.elevator.enablePID(false);
            RobotMap.elevator.elevatorManual(-power);
            holdPosition = RobotMap.elevatorEncoderWrapperDistance.pidGet();
        } else {
            if(RobotMap.elevatorEncoderWrapperDistance.pidGet() < 6.5 && (RobotMap.elevator.lastLevel == ElevatorLevel.FLOOR || RobotMap.elevator.lastLevel == ElevatorLevel.HATCH_INTAKE_FLOOR)) {
                RobotMap.elevator.enablePID(false);
            } else {
                RobotMap.elevator.enablePID(true);
                RobotMap.elevator.distanceSetpointTN.setOutput(holdPosition);
            }
       }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}