package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.CargoRollersIntake;
import org.usfirst.frc.team2485.robot.commands.Hook;
import org.usfirst.frc.team2485.robot.commands.Lift;
import org.usfirst.frc.team2485.robot.commands.Pushers;
import org.usfirst.frc.team2485.robot.commands.SetArmPosition;
import org.usfirst.frc.team2485.robot.commands.SetRollers;
import org.usfirst.frc.team2485.robot.commands.Slide;
import org.usfirst.frc.team2485.robot.subsystems.CargoArm;
import org.usfirst.frc.team2485.robot.subsystems.CargoRollers;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class CargoIntake extends CommandGroup {

    public CargoIntake(double power) {

        addSequential(new Hook(false));
        addSequential(new Pushers(false));
        addSequential(new Slide(false));
        addSequential(new Lift(false));
        
        // addParallel(new SetArmPosition(0));
        addSequential(new CargoRollersIntake(power));
        // addSequential(new SetArmPosition(CargoArm.TOP_POSITION));
    }

}
