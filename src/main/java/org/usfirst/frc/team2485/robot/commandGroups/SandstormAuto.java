package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.DriveStraight;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.Lift;
import org.usfirst.frc.team2485.robot.commands.SetAngle;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class SandstormAuto extends CommandGroup {
    AutoPath sideCargo, frontCargo;
    public SandstormAuto() {
        Pair[] sideCargoControlPoint = RobotMap.driveTrain.generateSandstormControlPoints(true);
        sideCargo = new AutoPath(AutoPath.getPointsForBezier(1000, new Pair (0,0), sideCargoControlPoint[0], RobotMap.driveTrain.getSandstormEndpoint(true)));
        addSequential(new DriveTo(sideCargo, 60, false, 15000, false, true));
        // addSequential(new PlaceHatch());
        // addSequential(new DriveStraight(-108, 5000));
        // addSequential(new SetAngle(-Math.PI/2, 2000));
        // addSequential(new Lift(true));
        // addSequential(new DriveStraight(245.52, 5000));
        // addSequential(new LoadingStationIntake());
        // addSequential(new DriveStraight(-(224*ConstantsIO.kPath), 5000));
        // addSequential(new SetAngle(Math.PI/2, 2000));
        // frontCargo = new AutoPath(AutoPath.getPointsForBezier(1000, new Pair(0,0), new Pair(0, 126), new Pair((224*(1.00-ConstantsIO.kPath)), 126)));
        // addSequential(new DriveTo(frontCargo, 60, false, 15000, false, true));
        // addSequential(new PlaceHatch());
        // addSequential(new DriveStraight(-10, 5000));


    }
}