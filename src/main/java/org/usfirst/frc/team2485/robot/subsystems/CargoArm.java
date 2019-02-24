package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class CargoArm extends Subsystem {

    public CargoArm() {

    }

    public void cargoArmManual(double power) {
       RobotMap.cargoArmPercentOutput.set(power);
    }

    public void initDefaultCommand() {

    }
}