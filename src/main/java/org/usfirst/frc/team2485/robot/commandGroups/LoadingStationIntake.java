package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.Hook;
import org.usfirst.frc.team2485.robot.commands.Lift;
import org.usfirst.frc.team2485.robot.commands.Slide;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class LoadingStationIntake extends CommandGroup {
    public LoadingStationIntake() {
        addSequential(new Lift(true));
        addSequential(new Slide(true));
        addSequential(new Hook(true));
        addSequential(new Hook(false));//add elevator movement

    }


}