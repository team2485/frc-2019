package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class SandstormAuto extends CommandGroup {
    AutoPath sideCargo;
    public SandstormAuto() {
        Pair[] sideCargoControlPoint = RobotMap.driveTrain.generateSandstormControlPoints(true);
        
        sideCargo = new AutoPath(AutoPath.getPointsForBezier(1000, new Pair (0,0), sideCargoControlPoint[0], RobotMap.driveTrain.getSandstormEndpoint(true)));
        addSequential(new DriveTo(sideCargo, 60, false, 15000, false, true));
        addSequential(new PlaceHatch());
    }
}