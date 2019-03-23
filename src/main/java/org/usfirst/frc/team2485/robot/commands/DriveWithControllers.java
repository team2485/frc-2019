package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.DriveTrain;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;
import org.usfirst.frc.team2485.util.ConstantsIO;

public class DriveWithControllers extends edu.wpi.first.wpilibj.command.Command {
  public DriveWithControllers() {
    setInterruptible(true);
    requires(RobotMap.driveTrain);
  }
  
  protected void initialize() {
    RobotMap.driveTrain.enableTeleopPID(true);
  }
  
  protected void execute() {
    double throttle = OI.getDriveThrottle();
    double steering = OI.getDriveSteering();
    boolean quickTurn = OI.getQuickTurn();
    boolean slowTurn = OI.jacket.getRawButton(OI.XBOX_B_PORT);
    
    if (!(quickTurn || slowTurn)) {
      steering = org.usfirst.frc.team2485.util.ThresholdHandler.deadbandAndScale(OI.jacket.getRawAxis(0), 0.2, 0.0, 1.0);
    }

    if (slowTurn) {
      steering *= 0.5;
    }

    double upRamp = ConstantsIO.teleopUpRampElevatorUp + (ConstantsIO.teleopUpRamp - ConstantsIO.teleopUpRampElevatorUp) * (ElevatorLevel.ROCKET_LEVEL_THREE.getPosition() - RobotMap.elevatorEncoderWrapperDistance.pidGet())/ElevatorLevel.ROCKET_LEVEL_THREE.getPosition();
    double downRamp = ConstantsIO.teleopDownRampElevatorUp + (ConstantsIO.teleopDownRamp - ConstantsIO.teleopDownRampElevatorUp) * (ElevatorLevel.ROCKET_LEVEL_THREE.getPosition() - RobotMap.elevatorEncoderWrapperDistance.pidGet())/ElevatorLevel.ROCKET_LEVEL_THREE.getPosition();

    RobotMap.driveTrain.teleopSetpointLeftRamp.setRampRates(upRamp, downRamp);
    RobotMap.driveTrain.teleopSetpointRightRamp.setRampRates(upRamp, downRamp);

    RobotMap.driveTrain.WarlordsDrive(throttle, steering, quickTurn || slowTurn);
  }
  
  protected boolean isFinished() {
    return false;
  }
}