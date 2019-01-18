package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

// eject non-sucky: middle one first, then wait .5 seconds, then side ones
//pull in non-sucky: middle one
//sucky on
//sucky off

public class HatchPanelIntake extends Subsystem {
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	
    }

    public static void suctionOn() {
        RobotMap.suctionSolenoid.set(true);
    }

    public static void suctionOff() {
        RobotMap.suctionSolenoid.set(false);
    }

    public static void centerOn() {
        RobotMap.centerSolenoidIn.set(false);
        RobotMap.centerSolenoidOut.set(true);                          
    }

    public static void centerOff() {
        RobotMap.centerSolenoidIn.set(true);
        RobotMap.centerSolenoidOut.set(false);        
    }

    public static void sideOn() {
        RobotMap.sideSolenoidIn.set(false);
        RobotMap.sideSolenoidOut.set(true);                
    }

    public static void sideOff() {
        RobotMap.sideSolenoidIn.set(true);
        RobotMap.sideSolenoidOut.set(false);
    }
        
    

    
}