package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;

import edu.wpi.first.wpilibj.command.Command;

public class CargoArmWithControllers extends Command {
    public CargoArmWithControllers() {
        setInterruptible(true);
        requires(RobotMap.cargoArm);
    }

    @Override
    protected void execute() {

        double power = ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_RYJOYSTICK_PORT), 0.2, 0, 0.6);

        if(power != 0 && !RobotMap.liftSolenoidOut.get()) {
            RobotMap.cargoArm.enablePID(false);
            RobotMap.cargoArm.cargoArmManual(-power);
            RobotMap.cargoArm.holdPosition = RobotMap.cargoArmEncoderWrapperDistance.pidGet();
        } else {
            RobotMap.cargoArm.enablePID(true);
            RobotMap.cargoArm.setPosition(RobotMap.cargoArm.holdPosition);
       }

       if(RobotMap.cargoArm.distancePID.isOnTarget() || RobotMap.cargoArmLimitSwitchUp.get()){
            RobotMap.cargoArm.enablePID(false);
            RobotMap.cargoArmTalonWrapperCurrent.set(RobotMap.cargoArm.HOLD_CURRENT);
       } 

       if(RobotMap.cargoArmLimitSwitchDown.get()){
           RobotMap.cargoArm.enablePID(false);
           RobotMap.cargoArmTalonWrapperCurrent.set(0);
       }



    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}