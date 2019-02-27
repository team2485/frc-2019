package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.Hook;
import org.usfirst.frc.team2485.robot.commands.Lift;
import org.usfirst.frc.team2485.robot.commands.Pushers;
import org.usfirst.frc.team2485.robot.commands.SetArmPosition;
import org.usfirst.frc.team2485.robot.commands.Slide;
import org.usfirst.frc.team2485.robot.commands.Wait;
import org.usfirst.frc.team2485.util.FinishedCondition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class PlaceHatch extends CommandGroup {
    public PlaceHatch(){
        addParallel(new SetArmPosition(1.8));
        addSequential(new Lift(true));
        addSequential(new Slide(true));
        addSequential(new Pushers(true));
        addSequential(new Hook(true));
        addSequential(new WaitCommand(2));
        addSequential(new Pushers(false));
        addSequential(new Hook(false));
        addSequential(new Slide(false));
        addSequential(new WaitCommand(0.25));
        addSequential(new Lift(false));
    }

}