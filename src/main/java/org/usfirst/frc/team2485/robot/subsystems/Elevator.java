package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class Elevator extends Subsystem {

    public Elevator() {

    }

    public void elevatorManual(double power) {
       RobotMap.elevatorPercentOutput.set(power);
    }

    public void initDefaultCommand() {

    }
}