package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;

public class SetArmPosition extends InstantCommand {
    
    private double position;
    public long time;

    public SetArmPosition(double position) {
        this.position = position;
        requires(RobotMap.cargoArm);
    }

    @Override
    protected void initialize() {
        time = System.currentTimeMillis();
        super.initialize();
        RobotMap.cargoArm.distanceSetpointTN.setOutput(position);
        RobotMap.cargoArm.distancePID.setAbsoluteTolerance(0.05);
        CargoArmWithControllers.power = position;
    }

   
}