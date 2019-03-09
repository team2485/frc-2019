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
        // double powerBackup = ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(1), 0.2, 0, 1);
        // power = power == 0 ? powerBackup : power;

        if(power != 0) {
        // if(power != 0 && !RobotMap.liftSolenoidOut.get()) {
            RobotMap.cargoArm.enablePID(false);
           // RobotMap.cargoArm.distanceSetpointTN.setOutput(power);
            RobotMap.cargoArm.cargoArmManual(-power);
           // RobotMap.cargoArm.holdPosition = RobotMap.cargoArmEncoderWrapperDistance.pidGet();
            // if(power > 0) {  
            //     RobotMap.cargoArm.holdPosition = 0;
            // } else {
            //     RobotMap.cargoArm.holdPosition = -1.7;

          //  }
            RobotMap.hatchIntake.hookIn();
            RobotMap.hatchIntake.retractPushers();
            RobotMap.hatchIntake.slideIn();
            RobotMap.hatchIntake.stow();
        } else if(RobotMap.cargoArmLimitSwitchUp.get() && Math.abs(RobotMap.cargoArm.distancePID.getError()) < 0.1) {
            RobotMap.cargoArm.enablePID(false);
            RobotMap.cargoArmTalonWrapperCurrent.set(CargoArm.HOLDING_CURRENT);
            RobotMap.cargoArm.holdPosition = 0;
        } else if(RobotMap.cargoArmLimitSwitchDown.get() && Math.abs(RobotMap.cargoArm.distancePID.getError()) < 0.1) {
            RobotMap.cargoArm.enablePID(false);
            RobotMap.cargoArmTalonWrapperPercentOutput.set(0);
            RobotMap.cargoArm.holdPosition = -1.7;
        } else {
            RobotMap.cargoArm.enablePID(true);
            RobotMap.cargoArm.setPosition(RobotMap.cargoArm.holdPosition);
        }

    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}