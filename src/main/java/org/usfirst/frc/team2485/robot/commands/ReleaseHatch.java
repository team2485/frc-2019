package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.HatchIntake;

import edu.wpi.first.wpilibj.command.Command;

public class ReleaseHatch extends InstantCommand {

    public ReleaseHatch() {
        //setInterruptible(true);
        requires(RobotMap.hatchIntake);
    }

    public static void init(){
        //button stuff 

        //RobotMap.hatchIntake.suctionOff();
        RobotMap.hatchIntake.sideOn();

        try {
            Thread.sleep(125);
        }
        catch (Exception e) {
            System.out.println("Failed");
        }

        RobotMap.hatchIntake.centerOn();

        try {
            Thread.sleep(250);
        }
        catch (Exception e) {
            System.out.println("Failed");
        }

        RobotMap.hatchIntake.sideOff();
        RobotMap.hatchIntake.centerOff();
        RobotMap.hatchIntake.pushOff(); // brings the hatch intake back in
 
    }

}