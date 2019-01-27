package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.HatchPanelIntake;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class HatchLoadingStationIntake extends InstantCommand {

    public HatchLoadingStationIntake() {
        //setInterruptible(true);
        requires(RobotMap.hatchPanelIntake);
    }

    public static void init(){
        RobotMap.hatchPanelIntake.suctionOff();
        RobotMap.hatchPanelIntake.sideOff();
        
        RobotMap.hatchPanelIntake.centerOn();

        try {
            Thread.sleep(500);
        }
        catch (Exception e) {
            System.out.println("Failed");
        }

        RobotMap.hatchPanelIntake.centerOff();

        

        
        
    }

}