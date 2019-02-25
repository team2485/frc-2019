package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.Hook;
import org.usfirst.frc.team2485.robot.commands.Lift;
import org.usfirst.frc.team2485.robot.commands.SetElevatorPosition;
import org.usfirst.frc.team2485.robot.commands.Slide;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class LoadingStationIntake extends CommandGroup {
    public LoadingStationIntake() {
        addSequential(new Lift(true));
        addSequential(new Slide(true));
        addSequential(new Hook(true));
        addSequential(new SetElevatorPosition(ElevatorLevel.HATCH_LIFTING));
        addSequential(new Hook(false));
        addSequential(new SetElevatorPosition(ElevatorLevel.FLOOR));

    }


}