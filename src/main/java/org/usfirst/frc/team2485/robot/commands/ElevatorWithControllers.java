package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.Elevator;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;

public class ElevatorWithControllers extends Command {
    public static double holdPosition = 0; 

    public ElevatorWithControllers() {
        setInterruptible(true);
        requires(RobotMap.elevator);
    }

    @Override
    protected void initialize() {
        // holdPosition = RobotMap.elevator.lastLevel.getPosition();
    }

    @Override
    protected void execute() {
        double power = OI.getElevatorManual();
        // double powerBackup = ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(5), 0.2, 0, 1);
        // power = power == 0 ? powerBackup : power;

        if(power != 0) {
            RobotMap.elevator.enablePID(false);
            RobotMap.elevator.elevatorManual(-power);
            holdPosition = RobotMap.elevatorEncoderWrapperDistance.pidGet();
            RobotMap.elevator.distanceSetpointTN.setOutput(holdPosition);
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