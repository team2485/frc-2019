package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.HatchPanelIntake;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class HatchRelease extends InstantCommand {

    public HatchRelease() {
        //setInterruptible(true);
        requires(RobotMap.hatchPanelIntake);
    }

    public static void init(){
        //button stuff 
        RobotMap.hatchPanelIntake.suctionOff();
        RobotMap.hatchPanelIntake.centerOff();

        try {
            Thread.sleep(250);
        }
        catch (Exception e) {
            System.out.println("Failed");
        }

        RobotMap.hatchPanelIntake.sideOn();

        try {
            Thread.sleep(250);
        }
        catch (Exception e) {
            System.out.println("Failed");
        }

        RobotMap.hatchPanelIntake.sideOff();
 
    }

}