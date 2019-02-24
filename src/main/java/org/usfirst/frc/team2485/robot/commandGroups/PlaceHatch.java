package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.Hook;
import org.usfirst.frc.team2485.robot.commands.Pushers;
import org.usfirst.frc.team2485.robot.commands.Slide;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class PlaceHatch extends CommandGroup {
    public PlaceHatch(){
        addSequential(new Slide(true));
        addSequential(new Pushers(true));
        addSequential(new Hook(true));
        addSequential(new Pushers(false));
        addSequential(new Hook(false));
    }

}