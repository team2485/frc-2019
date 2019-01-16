package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.HatchIntake;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class HatchLoadingStationIntake extends InstantCommand {

    public HatchLoadingStationIntake() {
        //setInterruptible(true);
        requires(RobotMap.hatchIntake);
    }

    public static void init(){
        HatchIntake.suctionOff();
        HatchIntake.sideOff();
        
        HatchIntake.centerOn();
        try {
            Thread.sleep(500);
        }
        catch (Exception e) {
            System.out.println("Failed");
        }

        

        
        
    }

}