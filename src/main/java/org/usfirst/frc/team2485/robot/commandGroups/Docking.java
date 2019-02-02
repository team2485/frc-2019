package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.util.AutoPath;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Docking extends CommandGroup {
    public Docking () {
        // if(RobotMap.driveTrain.meterRule(100) != null) {
        //     AutoPath.Pair[] meterRuleControlPoints = RobotMap.driveTrain.meterRule(100);
        //     AutoPath meterPath = new AutoPath((AutoPath.getPointsForBezier(100, meterRuleControlPoints[0], meterRuleControlPoints[1])));
        //     addSequential(new DriveTo(meterPath, 50, true, 30000, false, false));
        // }
        AutoPath.Pair endpoint = RobotMap.driveTrain.getAutoAlignEndpoint(100, 100, 120);
        AutoPath.Pair[] controlPoints = RobotMap.driveTrain.generateControlPoints(100, 100, 120);
        AutoPath path;
        if (controlPoints.length == 2) {
            path = new AutoPath(AutoPath.getPointsForBezier(1000, new AutoPath.Pair(0, 0), controlPoints[0], controlPoints[1], endpoint));
        } else {
            path = new AutoPath(AutoPath.getPointsForBezier(1000, new AutoPath.Pair(0, 0), controlPoints[0], endpoint));

        }
        addSequential(new DriveTo(path, 50, false, 30000, false, false));
    }
}