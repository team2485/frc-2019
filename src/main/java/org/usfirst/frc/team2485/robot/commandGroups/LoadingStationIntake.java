package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.Hook;
import org.usfirst.frc.team2485.robot.commands.Lift;
import org.usfirst.frc.team2485.robot.commands.SetArmPosition;
import org.usfirst.frc.team2485.robot.commands.SetElevatorPosition;
import org.usfirst.frc.team2485.robot.commands.Slide;
import org.usfirst.frc.team2485.robot.commands.Wait;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;
import org.usfirst.frc.team2485.util.FinishedCondition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitUntilCommand;

public class LoadingStationIntake extends CommandGroup {
    public LoadingStationIntake() {
        addParallel(new SetArmPosition(1.8));
        addSequential(new Lift(true));
        addSequential(new Slide(false));
        addSequential(new Hook(true));
        addSequential(new WaitCommand(.5));
        addSequential(new SetElevatorPosition(ElevatorLevel.HATCH_LIFTING));
        addSequential(new WaitCommand(1));
        addSequential(new Hook(false));
        addSequential(new WaitCommand(1));
        addSequential(new SetElevatorPosition(ElevatorLevel.ROCKET_LEVEL_ONE));

    }


}