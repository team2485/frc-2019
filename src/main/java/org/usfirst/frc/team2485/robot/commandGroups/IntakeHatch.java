package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.*;
import org.usfirst.frc.team2485.robot.subsystems.CargoArm;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;
import org.usfirst.frc.team2485.util.FinishedCondition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class IntakeHatch extends CommandGroup {
    public IntakeHatch(){
//        addSequential(new SetElevatorPosition(ElevatorLevel.HATCH_INTAKE_FLOOR));
        addSequential(new PrepareToIntake());
        addSequential(new SetHatchRollersPWM(-0.9));
//        addSequential(new HatchRollersIntake(0.8));
    }

}