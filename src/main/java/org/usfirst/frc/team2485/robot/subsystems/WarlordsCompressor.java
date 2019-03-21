package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.commands.CompressorController;

import edu.wpi.first.wpilibj.command.Subsystem;

public class WarlordsCompressor extends Subsystem {
    public double airRemaining;
    public static final double POUNDS_PER_MILLI = 2.4/1000;
    public double demand;
    public static final double CURRENT_THRESHOLD = 50;

    public WarlordsCompressor() {
        airRemaining = 120;
    }

    public void initDefaultCommand() {
        // setDefaultCommand(new CompressorController());
    }
}