package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

// eject non-sucky: middle one first, then wait .5 seconds, then side ones
//pull in non-sucky: middle one
//sucky on
//sucky off

public class HatchIntake extends Subsystem {
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	
    }

    // public static void suctionOn() {
    //     RobotMap.suctionSolenoid.set(true);
    // }

    // public static void suctionOff() {
    //     RobotMap.suctionSolenoid.set(false);
    // }

    public static void centerOn() {
        RobotMap.intakeCenterSolenoidIn.set(false);
        RobotMap.intakeCenterSolenoidOut.set(true);                          
    }

    public static void centerOff() {
        RobotMap.intakeCenterSolenoidIn.set(true);
        RobotMap.intakeCenterSolenoidOut.set(false);        
    }

    public static void sideOn() {
        RobotMap.intakeSideSolenoidIn.set(false);
        RobotMap.intakeSideSolenoidOut.set(true);                
    }

    public static void sideOff() {
        RobotMap.intakeSideSolenoidIn.set(true);
        RobotMap.intakeSideSolenoidOut.set(false);
    }

    public static void pushOn() {
        RobotMap.intakePushSolenoidIn.set(true);
        RobotMap.intakePushSolenoidOut.set(false);
    }

    public static void pushOff() {
        RobotMap.intakePushSolenoidIn.set(true);
        RobotMap.intakePushSolenoidOut.set(false);
    }

    public static void swivelOn() {
        RobotMap.intakeSwivelSolenoidIn.set(true);
        RobotMap.intakeSwivelSolenoidOut.set(false);
    }

    public static void swivelOff() {
        RobotMap.intakeSwivelSolenoidIn.set(true);
        RobotMap.intakeSwivelSolenoidOut.set(false);
    }


        
    

    
}