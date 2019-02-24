package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.ElevatorWithControllers;
import org.usfirst.frc.team2485.util.MotorSetter;
import org.usfirst.frc.team2485.util.RampRate;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Elevator extends Subsystem {

    public TransferNode distanceSetpointTN;
    public TransferNode distanceSetpointRampedTN;

    public RampRate distanceSetpointRampRate;

    public WarlordsPIDController distancePID;


    public Elevator() {
        distanceSetpointTN = new TransferNode(0);
        distanceSetpointRampedTN = new TransferNode(0);

        distanceSetpointRampRate = new RampRate();

        distancePID = new WarlordsPIDController();

        distanceSetpointRampRate.setSetpointSource(distanceSetpointTN);
        distanceSetpointRampRate.setOutputs(distanceSetpointRampedTN);

        distancePID.setSetpointSource(distanceSetpointRampedTN);
        distancePID.setOutputs(RobotMap.elevatorCurrent);

    }

    public void elevatorManual(double power) {
       RobotMap.elevatorPercentOutput.set(power);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new ElevatorWithControllers());
    }
}