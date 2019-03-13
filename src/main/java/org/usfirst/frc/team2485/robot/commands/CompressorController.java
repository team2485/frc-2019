package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Command;

public class CompressorController extends Command {
    public CompressorController() {
        requires(RobotMap.warlordsCompressor);
        setInterruptible(false);
    }

    @Override
    protected void initialize() {
        super.initialize();
        RobotMap.compressor.setClosedLoopControl(false);
    }

    @Override
    protected void execute() {
        super.execute();
        RobotMap.warlordsCompressor.demand += RobotMap.driveLeftTalon1.getOutputCurrent() + RobotMap.driveLeftTalon2.getOutputCurrent() + RobotMap.driveLeftTalon3.getOutputCurrent() + RobotMap.driveLeftTalon4.getOutputCurrent() + 
            RobotMap.driveRightTalon1.getOutputCurrent() +  RobotMap.driveRightTalon2.getOutputCurrent() +  RobotMap.driveRightTalon3.getOutputCurrent() +  RobotMap.driveRightTalon4.getOutputCurrent() +  
            RobotMap.elevatorTalon1.getOutputCurrent() + RobotMap.elevatorTalon2.getOutputCurrent() + RobotMap.cargoArmTalon.getOutputCurrent() + RobotMap.cargoRollersTalon.getOutputCurrent();
        if(RobotMap.warlordsCompressor.airRemaining <= 20) {
            RobotMap.compressor.setClosedLoopControl(true);
        } 
        if(RobotMap.compressor.enabled()) {
            RobotMap.warlordsCompressor.airRemaining += RobotMap.warlordsCompressor.POUNDS_PER_MILLI * 20;
        }
        if(RobotMap.warlordsCompressor.demand >= RobotMap.warlordsCompressor.CURRENT_THRESHOLD) {
            RobotMap.compressor.setClosedLoopControl(false);
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        super.end();
    }
}