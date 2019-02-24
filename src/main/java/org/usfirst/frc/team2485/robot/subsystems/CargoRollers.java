package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class CargoRollers extends Subsystem {

    public CargoRollers() {

    }

    public void cargoRollersManual(double power) {
       RobotMap.cargoRollersPercentOutput.set(power);
    }

    public void initDefaultCommand() {

    }
}