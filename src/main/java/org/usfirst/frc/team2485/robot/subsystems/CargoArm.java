package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.CargoArmWithControllers;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.FastMath;
import org.usfirst.frc.team2485.util.LowPassFilter;
import org.usfirst.frc.team2485.util.MotorSetter;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.RampRate;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.Subsystem;

public class CargoArm extends Subsystem {

    public static  double HOLDING_CURRENT = 5.5;
    public static final double TOP_POSITION = 1.7;
    public double holdPosition;

    public TransferNode distanceSetpointTN;
    public TransferNode distanceSetpointRampedTN;
    public TransferNode distanceOutputTN;
    public TransferNode armEncoderTN;
    public TransferNode failsafeTN;

    public PIDSourceWrapper armEncoderPIDSource;
    public PIDSourceWrapper distanceOutputPIDSource;

    public WarlordsPIDController distancePID;

    public LowPassFilter encoderFilter;

    public MotorSetter motorSetter;

    public RampRate distanceRampRate;


    public CargoArm() {
        armEncoderTN = new TransferNode(0);
        distanceSetpointTN = new TransferNode(0);
        distanceSetpointRampedTN = new TransferNode(0);
        distanceOutputTN = new TransferNode(0);
        failsafeTN = new TransferNode(0);

        armEncoderPIDSource = new PIDSourceWrapper();
        distanceOutputPIDSource = new PIDSourceWrapper();

        distancePID = new WarlordsPIDController();

        encoderFilter = new LowPassFilter();

        motorSetter = new MotorSetter();
        distanceRampRate = new RampRate();

        distanceRampRate.setSetpointSource(distanceSetpointTN);
        distanceRampRate.setOutputs(distanceSetpointRampedTN);

        encoderFilter.setFilterCoefficient(ConstantsIO.kEncoderFilterCoefficient);
        encoderFilter.setSetpointSource(RobotMap.cargoArmEncoder);
        encoderFilter.setOutputs(armEncoderTN);

        armEncoderPIDSource.setPidSource(() -> {
            return armEncoderTN.pidGet();
        });

        distancePID.setSetpointSource(distanceSetpointRampedTN); 
        distancePID.setOutputs(distanceOutputTN);
        distancePID.setSources(armEncoderPIDSource);
        distancePID.setOutputRange(ConstantsIO.cargoArmIMaxDown, ConstantsIO.cargoArmIMaxUp);
       

        distanceOutputPIDSource.setPidSource(() -> {
            double output = distanceOutputTN.getOutput() + ConstantsIO.levitateCargo * FastMath.cos(RobotMap.cargoArmEncoderWrapperDistance.pidGet());
            if(failsafeTN.getOutput() != 0) {
                return failsafeTN.getOutput();
            } else if(distanceSetpointTN.getOutput() < RobotMap.cargoArmEncoderWrapperDistance.pidGet()){
                if(output > ConstantsIO.cargoArmIMaxDown){
                    return ConstantsIO.cargoArmIMaxDown;
                } else if (output < -ConstantsIO.cargoArmIMaxDown){
                    return -ConstantsIO.cargoArmIMaxDown;
                } else { 
                    return output;
                }
            } else if (distanceSetpointTN.getOutput() > RobotMap.cargoArmEncoderWrapperDistance.pidGet()){
                if(output > ConstantsIO.cargoArmIMaxUp){
                    return ConstantsIO.cargoArmIMaxUp;
                } else if (output < -ConstantsIO.cargoArmIMaxUp){
                    return -ConstantsIO.cargoArmIMaxUp;
                } else { 
                    return output;
                }
            } else { 
                return 0;
            }
            
        });

        

        motorSetter.setSetpointSource(distanceOutputPIDSource);
        motorSetter.setOutputs(RobotMap.cargoArmCurrent);

        holdPosition = 0; 
       
    }


    public void cargoArmManual(double power) {
       RobotMap.cargoArmPercentOutput.set(power);
    }

    public void initDefaultCommand() {
       setDefaultCommand(new CargoArmWithControllers());
    }

    public void setPosition(double position) {
        distanceSetpointTN.setOutput(position);
    }

    public void updateConstants() {
        distanceRampRate.setRampRates(ConstantsIO.armDistanceSetpointUpRamp,ConstantsIO.armDistanceSetpointDownRamp);
        distancePID.setPID(ConstantsIO.kP_cargoArmDistance, ConstantsIO.kI_cargoArmDistance, ConstantsIO.kD_cargoArmDistance);
    }

    public void enablePID(boolean enabled) {
        if (enabled) {
            distancePID.enable();
            motorSetter.enable();
            distanceRampRate.enable();
        } else {
            distanceOutputTN.setOutput(0);
            distancePID.disable();
            motorSetter.disable();
        }
    }


}