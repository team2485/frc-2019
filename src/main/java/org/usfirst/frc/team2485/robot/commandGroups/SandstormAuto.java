package org.usfirst.frc.team2485.robot.commandGroups;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.DriveStraight;
import org.usfirst.frc.team2485.robot.commands.DriveTo;
import org.usfirst.frc.team2485.robot.commands.Lift;
import org.usfirst.frc.team2485.robot.commands.SetAngle;
import org.usfirst.frc.team2485.robot.commands.SetArmPosition;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class SandstormAuto extends CommandGroup {
    public static AutoPath sideCargo, frontCargo;
    public SandstormAuto() {
        // Pair[] sideCargoControlPoint = RobotMap.driveTrain.generateSandstormControlPoints(true);
        // sideCargo = new AutoPath(AutoPath.getPointsForBezier(1000, new Pair (0,0), sideCargoControlPoint[0], RobotMap.driveTrain.getSandstormEndpoint(true)));
        // addSequential(new DriveTo(sideCargo, 60, false, 15000, false, true));
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
        addSequential(new SetArmPosition(0));
        addSequential(new DriveStraight(100, 5000)); //go off platform
        addSequential(new DriveStraight(-100, 5000)); //by here, we should stall against side of lvl 2 platform and drive normal 3 pt. path
        addSequential(new DriveTo(sideCargo, 70, false, 15000, false, false));
        // addSequential(new DriveTo(frontCargo, 70, false, 15000, false, false));
        addSequential(new PlaceHatch());


    }

    public static void init(boolean left) {
        int sign = left ? 1 : -1;
        sideCargo = new AutoPath(AutoPath.getPointsForBezier(1000, new Pair(0, 0), new Pair(0, 214), new Pair(sign*35.5, 214)));
        frontCargo = new AutoPath(AutoPath.getPointsForBezier(1000, new Pair(0,0), new Pair(0, 174)));
    }

}