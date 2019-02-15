package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.SetAngle;
import org.usfirst.frc.team2485.util.AutoPath;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;

public class Docking extends CommandGroup {
    public Docking () {
        // addSequential(new SetAngle(Math.PI/3, Math.toRadians(3)));

        // if(RobotMap.driveTrain.meterRule(100) != null) {
        //     AutoPath.Pair[] meterRuleControlPoints = RobotMap.driveTrain.meterRule(100);
        //     AutoPath meterPath = new AutoPath((AutoPath.getPointsForBezier(100, meterRuleControlPoints[0], meterRuleControlPoints[1])));
        //     addSequential(new DriveTo(meterPath, 50, true, 30000, false, false));
        // }
       // AutoPath.Pair endpoint = RobotMap.driveTrain.getAutoAlignEndpoint(100, 70, 100);
        AutoPath.Pair endpoint = RobotMap.driveTrain.getAutoAlignEndpoint(RobotMap.lidar.getDistanceCorrected(), 90, 110);
        AutoPath.Pair[] controlPoints = RobotMap.driveTrain.generateControlPoints(RobotMap.lidar.getDistanceCorrected(), 90, 110);
        System.out.println("Docking");
        System.out.println("CP 1: " + controlPoints[0].getX() + ", " + controlPoints[0].getY());
		System.out.println("CP 2: " + controlPoints[1].getX() + ", " + controlPoints[1].getY());
        AutoPath path;
        if (controlPoints.length == 2) {
            path = new AutoPath(AutoPath.getPointsForBezier(1000, new AutoPath.Pair(0, 0), controlPoints[0], controlPoints[1], endpoint));
        } else {
            path = new AutoPath(AutoPath.getPointsForBezier(1000, new AutoPath.Pair(0, 0), controlPoints[0], endpoint));

        }
        System.out.println("Len CP: " + controlPoints.length);
        addSequential(new DriveTo(path, 50, false, 30000, false, false));
    }
}