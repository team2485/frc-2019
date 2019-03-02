package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.AdjustElevatorPosition;
import org.usfirst.frc.team2485.robot.commands.Hook;
import org.usfirst.frc.team2485.robot.commands.Lift;
import org.usfirst.frc.team2485.robot.commands.Pushers;
import org.usfirst.frc.team2485.robot.commands.SetArmPosition;
import org.usfirst.frc.team2485.robot.commands.SetElevatorPosition;
import org.usfirst.frc.team2485.robot.commands.Slide;
import org.usfirst.frc.team2485.robot.commands.Wait;
import org.usfirst.frc.team2485.robot.subsystems.CargoArm;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;
import org.usfirst.frc.team2485.util.FinishedCondition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class PlaceHatch extends CommandGroup {
    public PlaceHatch(){
        addParallel(new SetArmPosition(CargoArm.TOP_POSITION));
        addSequential(new Lift(true));
        addSequential(new Slide(true));
        addSequential(new Hook(true));
        addSequential(new WaitCommand(.05));
        addSequential(new Pushers(true));
        addSequential(new WaitCommand(0.25));
        addSequential(new Pushers(false));
        addSequential(new AdjustElevatorPosition(-3));
        addSequential(new WaitCommand(0.5));
        addSequential(new Hook(false));
        addSequential(new Slide(false));
        // addSequential(new WaitCommand(0.25));
    }

}