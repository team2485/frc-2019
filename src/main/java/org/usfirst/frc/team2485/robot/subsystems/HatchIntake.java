package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;

public class HatchIntake extends Subsystem {

    public HatchIntake() {
    }

    public void lift() {
        RobotMap.liftSolenoidIn.set(false);
        RobotMap.liftSolenoidOut.set(true);
    }

    public void stow() {
        RobotMap.liftSolenoidIn.set(true);
        RobotMap.liftSolenoidOut.set(false);
    }

    public void slideOut() {
        RobotMap.slideSolenoidIn.set(false);
        RobotMap.slideSolenoidOut.set(true);
    }

    public void slideIn() {
        RobotMap.slideSolenoidIn.set(true);
        RobotMap.slideSolenoidOut.set(false);
    }

   

    public void initDefaultCommand() {

    }
}