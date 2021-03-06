package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.AdjustElevatorPosition;
import org.usfirst.frc.team2485.robot.commands.HoldingCurrent;
import org.usfirst.frc.team2485.robot.commands.Hook;
import org.usfirst.frc.team2485.robot.commands.Lift;
import org.usfirst.frc.team2485.robot.commands.SetArmPosition;
import org.usfirst.frc.team2485.robot.commands.SetElevatorPosition;
import org.usfirst.frc.team2485.robot.commands.SetHatchRollersPWM;
import org.usfirst.frc.team2485.robot.commands.Slide;
import org.usfirst.frc.team2485.robot.commands.Wait;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;
import org.usfirst.frc.team2485.util.FinishedCondition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitUntilCommand;

public class LoadingStationIntake extends CommandGroup {
    public LoadingStationIntake() {
        RobotMap.compressor.setClosedLoopControl(false);
        addSequential(new SetArmPosition(0));
        addSequential(new Lift(true));
        addSequential(new Slide(false));
        addSequential(new SetHatchRollersPWM(-0.2)); //is this negative or positive???
        addSequential(new SetElevatorPosition(ElevatorLevel.HATCH_LIFTING));
        // addSequential(new SetElevatorPosition(ElevatorLevel.ROCKET_LEVEL_ONE));

    }


}