package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.commands.CargoArmWithControllers;
import org.usfirst.frc.team2485.robot.commands.CargoRollersIntake;
import org.usfirst.frc.team2485.robot.commands.Hook;
import org.usfirst.frc.team2485.robot.commands.Lift;
import org.usfirst.frc.team2485.robot.commands.Pushers;
import org.usfirst.frc.team2485.robot.commands.SetArmPosition;
import org.usfirst.frc.team2485.robot.commands.SetRollers;
import org.usfirst.frc.team2485.robot.commands.Slide;
import org.usfirst.frc.team2485.robot.subsystems.CargoArm;
import org.usfirst.frc.team2485.robot.subsystems.CargoRollers;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class CargoIntake extends CommandGroup {

    public CargoIntake(double power) {

        addSequential(new Hook(false));
        addSequential(new Pushers(false));
        addSequential(new Slide(false));
        addSequential(new Lift(false));
        
        //addSequential(new SetArmPosition(-1.9));
        
        CommandGroup lowerAndSpin = new CommandGroup();
        lowerAndSpin.addParallel(new CargoArmWithControllers());
        lowerAndSpin.addParallel(new CargoRollersIntake(power));

        addSequential(lowerAndSpin);
    }

}
