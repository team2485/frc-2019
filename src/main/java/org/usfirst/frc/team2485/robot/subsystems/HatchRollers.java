package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class HatchRollers extends Subsystem {

    public static final double HOLDING_CURRENT = 1;
    public boolean intaken;

    public HatchRollers(){
        intaken = false;
    }

    public void hatchRollersManual(double power) {
        RobotMap.hatchRollersPercentOutput.set(power);
     }

    public void hatchRollersCurrent(double current) {
        RobotMap.hatchRollersCurrent.set(current);
    }


    @Override
    protected void initDefaultCommand() {

    }

}