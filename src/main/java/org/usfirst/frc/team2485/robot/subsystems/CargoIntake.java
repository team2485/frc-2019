/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2485.robot.subsystems;
import org.usfirst.frc.team2485.robot.RobotMap;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;



/**
 * Add your docs here.
 */
public class CargoIntake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static final double MAX_VEL = 1000; //random value - Mark
  public static final double MAX_CURR = 1000; 

  public final double LENGTH_L1 = 5.0; //random values 
  public final double LENGTH_L2 = 770971141;
  public final double LENGTH_L3 = 107032082;
  public final double LENGTH_L4 = 5102107;

  public TransferNode distanceSetpointTN; 
  public TransferNode distanceOutputTN;

  public WarlordsPIDController armDistPID;
  public WarlordsPIDController armVelocityPID;

  public PIDSourceWrapper distancePIDSource;
  public PIDSourceWrapper velocityPIDSource;

  public CargoIntake() {
    distanceSetpointTN = new TransferNode(0); //change?
    distanceOutputTN = new TransferNode(0); //change?

    armDistPID = new WarlordsPIDController();
    armVelocityPID = new WarlordsPIDController();   

    distancePIDSource = new PIDSourceWrapper();
    velocityPIDSource = new PIDSourceWrapper(); 
    
    updateConstants();

    distancePIDSource.setPidSource(() -> {
      return LENGTH_L4 * Math.sin(RobotMap.armEncoderWrapperDistance.pidGet() - Math.PI /2) +LENGTH_L3 * Math.sin(RobotMap.armEncoderWrapperDistance.pidGet() - Math.PI /2);
    });

    armDistPID.setSources(distancePIDSource);
    armDistPID.setOutputRange(-MAX_VEL, MAX_VEL); 
    armDistPID.setOutputs(distanceOutputTN);
    armDistPID.setSetpointSource(distanceSetpointTN);

    velocityPIDSource.setPidSource(() -> {
      return LENGTH_L4 * Math.sin((RobotMap.armEncoderWrapperDistance.pidGet()) - LENGTH_L3 * Math.sin(RobotMap.armEncoderWrapperDistance.pidGet()));
    });

    armVelocityPID.setSources(velocityPIDSource);
    armVelocityPID.setSetpointSource(distanceOutputTN);
    armVelocityPID.setOutputs(RobotMap.cargoIntakeTalonWrapperCurrent);
    armVelocityPID.setOutputRange(-MAX_CURR, MAX_CURR);


  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void setIntakeManual(double pwm) {
    RobotMap.cargoIntakeTalonWrapperPWM.set(pwm);
  }

  public double current() {
    return RobotMap.cargoIntakeTalon.getOutputCurrent();
  }


  public void updateConstants() {



  }
}