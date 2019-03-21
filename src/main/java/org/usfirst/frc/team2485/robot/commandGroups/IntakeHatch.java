package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.AdjustElevatorPosition;
import org.usfirst.frc.team2485.robot.commands.ElevatorWithControllers;
import org.usfirst.frc.team2485.robot.commands.HatchRollersIntake;
import org.usfirst.frc.team2485.robot.commands.HoldingCurrent;
import org.usfirst.frc.team2485.robot.commands.Hook;
import org.usfirst.frc.team2485.robot.commands.Lift;
import org.usfirst.frc.team2485.robot.commands.Pushers;
import org.usfirst.frc.team2485.robot.commands.SetArmPosition;
import org.usfirst.frc.team2485.robot.commands.SetElevatorPosition;
import org.usfirst.frc.team2485.robot.commands.SetHatchRollersPWM;
import org.usfirst.frc.team2485.robot.commands.Slide;
import org.usfirst.frc.team2485.robot.commands.Wait;
import org.usfirst.frc.team2485.robot.subsystems.CargoArm;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;
import org.usfirst.frc.team2485.util.FinishedCondition;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class IntakeHatch extends CommandGroup {
    public IntakeHatch(){
        addSequential(new SetElevatorPosition(ElevatorLevel.HATCH_INTAKE_FLOOR));
        addSequential(new HatchRollersIntake(0.8));
    }

}