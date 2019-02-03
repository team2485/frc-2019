/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.MotorSetter;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static final double MAX_VEL = 10;
  private static final double MAX_CURR = 30;

  public TransferNode distanceSetpointTN; // what do we name this?
  public TransferNode distanceOutputTN;

  public WarlordsPIDController elevatorDistPID;
  public WarlordsPIDController elevatorVelocityPID;

  public PIDSourceWrapper distancePIDSource;
  public PIDSourceWrapper velocityPIDSource;

  public MotorSetter leftMotorSetter;
  public MotorSetter rightMotorSetter;

  double elevatorLH; // lower hatch
  double elevatorMH; // middle hatch
  double elevatorUH; // upper hatch
  double elevatorLC; // lower cargo
  double elevatorMC; // middle cargo
  double elevatorUC; // upper cargo

  public Elevator() {

    distanceSetpointTN = new TransferNode(0);
    distanceOutputTN = new TransferNode(0);

    elevatorDistPID = new WarlordsPIDController();
    elevatorVelocityPID = new WarlordsPIDController();   

    distancePIDSource = new PIDSourceWrapper();
    velocityPIDSource = new PIDSourceWrapper();        

    leftMotorSetter = new MotorSetter();
    rightMotorSetter = new MotorSetter();

    updateConstants();

    distancePIDSource.setPidSource(() -> {
      return (RobotMap.elevatorEncoderWrapperDistance.pidGet());
    });

    elevatorDistPID.setSources(distancePIDSource);
    elevatorDistPID.setOutputRange(-MAX_VEL, MAX_VEL);
    elevatorDistPID.setOutputs(distanceOutputTN);
    elevatorDistPID.setSetpointSource(distanceSetpointTN);

    velocityPIDSource.setPidSource(() -> {
      return (RobotMap.elevatorEncoderWrapperRate.pidGet());
    });

    elevatorVelocityPID.setSources(velocityPIDSource);
    elevatorVelocityPID.setSetpointSource(distanceOutputTN);
    elevatorVelocityPID.setOutputs(RobotMap.elevatorWrapperCurrent);
    elevatorVelocityPID.setOutputRange(-MAX_CURR, MAX_CURR);



  }

  public void updateConstants() {



  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public static void movePWM() {
    RobotMap.elevatorTalonWrapperPWM1.set(0.5);
    RobotMap.elevatorTalonWrapperPWM2.set(0.5);
  }

}
