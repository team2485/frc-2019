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

    if(RobotMap.elevatorEncoderWrapperDistance.pidGet() >= ElevatorLevel.ROCKET_LEVEL_TWO.getPosition()) {
      double upRamp = ConstantsIO.teleopUpRampElevatorUp + (ConstantsIO.teleopUpRamp - ConstantsIO.teleopUpRampElevatorUp) * (ElevatorLevel.ROCKET_LEVEL_THREE.getPosition() - RobotMap.elevatorEncoderWrapperDistance.pidGet())/ElevatorLevel.ROCKET_LEVEL_THREE.getPosition();
      double downRamp = ConstantsIO.teleopDownRampElevatorUp + (ConstantsIO.teleopDownRamp - ConstantsIO.teleopDownRampElevatorUp) * (ElevatorLevel.ROCKET_LEVEL_THREE.getPosition() - RobotMap.elevatorEncoderWrapperDistance.pidGet())/ElevatorLevel.ROCKET_LEVEL_THREE.getPosition();
  
      RobotMap.driveTrain.teleopSetpointLeftRamp.setRampRates(upRamp, downRamp);
      RobotMap.driveTrain.teleopSetpointRightRamp.setRampRates(upRamp, downRamp);
    } else {
      double upRampA = ConstantsIO.teleopUpRampQ;
      double downRampA = ConstantsIO.teleopDownRampQ;

      double upRampB = ConstantsIO.teleopUpRampL;
      double downRampB = ConstantsIO.teleopDownRampL;

      RobotMap.driveTrain.teleopSetpointLeftRamp.setRampRates(upRampA, upRampB, downRampA, downRampB);
      RobotMap.driveTrain.teleopSetpointRightRamp.setRampRates(upRampA, upRampB, downRampA, downRampB);
    }

    // if (RobotMap.driveTrain.teleopSetpointLeftRamp.isQuadratic() && RobotMap.driveTrain.teleopSetpointRightRamp.isQuadratic()) {
    //   double upRampA = ConstantsIO.teleopUpRampLElevatorUp + (ConstantsIO.teleopUpRampL - ConstantsIO.teleopUpRampLElevatorUp) * (ElevatorLevel.ROCKET_LEVEL_THREE.getPosition() - RobotMap.elevatorEncoderWrapperDistance.pidGet())/ElevatorLevel.ROCKET_LEVEL_THREE.getPosition();
    //   double downRampA = ConstantsIO.teleopDownRampLElevatorUp + (ConstantsIO.teleopDownRampL - ConstantsIO.teleopDownRampLElevatorUp) * (ElevatorLevel.ROCKET_LEVEL_THREE.getPosition() - RobotMap.elevatorEncoderWrapperDistance.pidGet())/ElevatorLevel.ROCKET_LEVEL_THREE.getPosition();
  
    //   double upRampB = ConstantsIO.teleopUpRampQElevatorUp + (ConstantsIO.teleopUpRampQ - ConstantsIO.teleopUpRampQElevatorUp) * (ElevatorLevel.ROCKET_LEVEL_THREE.getPosition() - RobotMap.elevatorEncoderWrapperDistance.pidGet())/ElevatorLevel.ROCKET_LEVEL_THREE.getPosition();
    //   double downRampB = ConstantsIO.teleopDownRampQElevatorUp + (ConstantsIO.teleopDownRampQ - ConstantsIO.teleopDownRampQElevatorUp) * (ElevatorLevel.ROCKET_LEVEL_THREE.getPosition() - RobotMap.elevatorEncoderWrapperDistance.pidGet())/ElevatorLevel.ROCKET_LEVEL_THREE.getPosition();

    //   RobotMap.driveTrain.teleopSetpointLeftRamp.setRampRates(upRampA, upRampB, downRampA, downRampB);
    //   RobotMap.driveTrain.teleopSetpointRightRamp.setRampRates(upRampA, upRampB, downRampA, downRampB);

    // } else {
    //   double upRamp = ConstantsIO.teleopUpRampElevatorUp + (ConstantsIO.teleopUpRamp - ConstantsIO.teleopUpRampElevatorUp) * (ElevatorLevel.ROCKET_LEVEL_THREE.getPosition() - RobotMap.elevatorEncoderWrapperDistance.pidGet())/ElevatorLevel.ROCKET_LEVEL_THREE.getPosition();
    //   double downRamp = ConstantsIO.teleopDownRampElevatorUp + (ConstantsIO.teleopDownRamp - ConstantsIO.teleopDownRampElevatorUp) * (ElevatorLevel.ROCKET_LEVEL_THREE.getPosition() - RobotMap.elevatorEncoderWrapperDistance.pidGet())/ElevatorLevel.ROCKET_LEVEL_THREE.getPosition();
  
    //   RobotMap.driveTrain.teleopSetpointLeftRamp.setRampRates(upRamp, downRamp);
    //   RobotMap.driveTrain.teleopSetpointRightRamp.setRampRates(upRamp, downRamp);
    // }
   
   RobotMap.driveTrain.WarlordsDrive(throttle, steering, quickTurn || slowTurn);
  }
  
  protected boolean isFinished() {
    return false;
  }
}