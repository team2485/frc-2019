package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.CargoArmWithControllers;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.FastMath;
import org.usfirst.frc.team2485.util.MotorSetter;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.Subsystem;

public class CargoArm extends Subsystem {

    public static final double HOLD_CURRENT = 0.5;
    public double holdPosition;

    public TransferNode distanceSetpointTN;
    public TransferNode distanceOutputTN;

    public PIDSourceWrapper distanceOutputPIDSource;

    public WarlordsPIDController distancePID;

    public MotorSetter motorSetter;


    public CargoArm() {
        distanceSetpointTN = new TransferNode(0);
        distanceOutputTN = new TransferNode(0);

        distanceOutputPIDSource = new PIDSourceWrapper();

        distancePID = new WarlordsPIDController();

        motorSetter = new MotorSetter();


        distancePID.setSetpointSource(distanceSetpointTN);
        distancePID.setOutputs(distanceOutputTN);
        distancePID.setSources(RobotMap.cargoArmEncoderWrapperDistance);
        distancePID.setOutputRange(-ConstantsIO.cargoArmIMax, ConstantsIO.cargoArmIMax);

        distanceOutputPIDSource.setPidSource(() -> {
            return (distanceOutputTN.getOutput() + ConstantsIO.levitateCargo * FastMath.cos(RobotMap.cargoArmEncoderWrapperDistance.pidGet()));
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
        distancePID.setPID(ConstantsIO.kP_cargoArmDistance, ConstantsIO.kI_cargoArmDistance, ConstantsIO.kD_cargoArmDistance);
    }

    public void enablePID(boolean enabled) {
        if (enabled) {
            distancePID.enable();
            motorSetter.enable();
        } else {
            distanceSetpointTN.setOutput(0);
            distanceOutputTN.setOutput(0);
            distancePID.disable();
            motorSetter.disable();
        }
    }


}