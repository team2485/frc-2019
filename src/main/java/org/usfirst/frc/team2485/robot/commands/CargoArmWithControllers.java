package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.CargoArm;
import org.usfirst.frc.team2485.util.FastMath;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;

public class CargoArmWithControllers extends Command {
    public CargoArmWithControllers() {
        setInterruptible(true);
        requires(RobotMap.cargoArm);
    }

    @Override
    protected void execute() {

        double power = OI.getArmManual();

        if(power != 0) {
        // if(power != 0 && !RobotMap.liftSolenoidOut.get()) {
            RobotMap.cargoArm.enablePID(false);
            RobotMap.cargoArm.cargoArmManual(-power);
            RobotMap.cargoArm.holdPosition = RobotMap.cargoArmEncoderWrapperDistance.pidGet();
            RobotMap.hatchIntake.hookIn();
            RobotMap.hatchIntake.retractPushers();
            RobotMap.hatchIntake.slideIn();
            RobotMap.hatchIntake.stow();
        } else if(RobotMap.cargoArmLimitSwitchUp.get()) {
            RobotMap.cargoArm.enablePID(false);
            RobotMap.cargoArmTalonWrapperCurrent.set(2);
        } else if(RobotMap.cargoArmLimitSwitchDown.get()) {
            RobotMap.cargoArm.enablePID(false);
            RobotMap.cargoArmTalonWrapperCurrent.set(0);
        } 
        else {
            RobotMap.cargoArm.enablePID(true);
            RobotMap.cargoArm.setPosition(RobotMap.cargoArm.holdPosition);
        }

    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}