package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.HatchRollersIntake;
import org.usfirst.frc.team2485.robot.commands.SetHatchRollersPWM;
import org.usfirst.frc.team2485.robot.commands.SetHatchRollersPWMInstant;
import org.usfirst.frc.team2485.robot.commands.Slide;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class SandstormHatchClaimer extends CommandGroup {
    public SandstormHatchClaimer() {
        addSequential(new SetHatchRollersPWMInstant(-0.9));
        addSequential(new WaitCommand(0.7));
        CommandGroup extendAndIntake = new CommandGroup();
        addParallel(new SetHatchRollersPWMInstant(-0.1));
        addParallel(new Slide(true));
        addSequential(extendAndIntake);
    }
}