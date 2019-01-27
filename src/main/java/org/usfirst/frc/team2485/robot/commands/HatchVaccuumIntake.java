package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.HatchPanelIntake;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class HatchVaccuumIntake extends InstantCommand {

    public HatchVaccuumIntake() {
        //setInterruptible(true);
        requires(RobotMap.hatchPanelIntake);
    }

    public static void init(){
        //button stuff 
        RobotMap.hatchPanelIntake.suctionOn();
        RobotMap.hatchPanelIntake.centerOff();
        RobotMap.hatchPanelIntake.sideOff();
    }

}