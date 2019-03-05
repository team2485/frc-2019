package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class SetArmPosition extends Command {
    
    private double position;
    public long time;

    public SetArmPosition(double position) {
        this.position = position;
        requires(RobotMap.cargoArm);
        setInterruptible(true);
    }

    @Override
    protected void initialize() {
        time = System.currentTimeMillis();
        super.initialize();
        RobotMap.cargoArm.setPosition(position);
        RobotMap.cargoArm.distancePID.setAbsoluteTolerance(0.05);
        if(RobotMap.liftSolenoidOut.get() == false) {
            RobotMap.cargoArm.enablePID(true);
            
        } else {
            System.out.println("Cannot lower cargo arm, hatch lifted");
        }
    }

    @Override
    protected void execute() {
        super.execute();
    }

    @Override
    protected boolean isFinished() {
        return RobotMap.cargoArm.distancePID.isOnTarget() || System.currentTimeMillis() - time > 15000;
    }

    @Override
    protected void end() {
        if (RobotMap.cargoArmLimitSwitchDown.get()) {
			RobotMap.cargoArmEncoder.reset();            
        } else if (RobotMap.cargoArmLimitSwitchUp.get()) {

        }
        RobotMap.cargoArm.enablePID(false);
        super.end();
    }
}