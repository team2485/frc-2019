package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.util.AutoPath;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Docking extends CommandGroup {
    public Docking () {
        // if(RobotMap.driveTrain.meterRule() != null) {
        //     addSequential(new DriveTo(AutoPath.getPointsForBezier(100, RobotMap.driveTrain.meterRule())), 50, true, 30000, false, false));
        // }
        // addSequential(new DriveTo(AutoPath.getPointsForBezier(1000, RobotMap.driveTrain.generateControlPoints()), 50, false, 30000, false, false));
    }
}