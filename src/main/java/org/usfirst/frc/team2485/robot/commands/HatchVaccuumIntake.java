package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.HatchIntake;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class HatchVaccuumIntake extends InstantCommand {

    public HatchVaccuumIntake() {
        //setInterruptible(true);
        requires(RobotMap.hatchIntake);
    }

    public static void init(){
        //button stuff 
        RobotMap.hatchIntake.suctionOn();
        RobotMap.hatchIntake.centerOff();
        RobotMap.hatchIntake.sideOff();
    }

}