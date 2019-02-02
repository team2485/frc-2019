package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.HatchIntake;

import edu.wpi.first.wpilibj.command.InstantCommand;

public class HatchGroundIntake extends InstantCommand {

    public HatchGroundIntake() {
        //setInterruptible(true);
        requires(RobotMap.hatchIntake);
    }

    public static void init(){
        RobotMap.hatchIntake.centerOff();
        RobotMap.hatchIntake.sideOff();

        RobotMap.hatchIntake.suctionOn();

        //arm stuff here

        RobotMap.hatchIntake.centerOn();
        RobotMap.hatchIntake.suctionOff();

        try {
            Thread.sleep(250);
        } catch (Exception e) {
            System.out.println("Failed");
        }

        RobotMap.hatchIntake.centerOff();



        
    }

}