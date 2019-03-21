package org.usfirst.frc.team2485.robot.commands;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.DriveTrain;

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
    
    if (!quickTurn) {
      steering = org.usfirst.frc.team2485.util.ThresholdHandler.deadbandAndScale(OI.jacket.getRawAxis(0), 0.2, 0.0, 1.0);
    }

    RobotMap.driveTrain.WarlordsDrive(throttle, steering, quickTurn);
  }
  
  protected boolean isFinished() {
    return false;
  }
}