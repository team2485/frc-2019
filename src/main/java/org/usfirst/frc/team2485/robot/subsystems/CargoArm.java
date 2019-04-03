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
import org.usfirst.frc.team2485.util.WarlordsPIDControllerSystem;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.Subsystem;

public class CargoArm extends Subsystem {

    public static final double STARTING_CURRENT = 7;
    public static  final double HOLDING_CURRENT = 3;
    public double holdPosition;

    public TransferNode distanceSetpointTN = new TransferNode(0.0);
    public TransferNode distanceSetpointRampedTN = new TransferNode(0.0);
    public TransferNode armEncoderTN = new TransferNode(0.0);
    public TransferNode failsafeTN = new TransferNode(0.0);
    public TransferNode distanceOutputTN = new TransferNode(0.0);
    public TransferNode distanceOutputFilteredTN = new TransferNode(0.0);
    public TransferNode armMaxOutputTN = new TransferNode(0.0);
    public TransferNode armMinOutputTN = new TransferNode(0.0);
    public LowPassFilter armEncoderFilter = new LowPassFilter();
    public LowPassFilter distanceOutputFilter = new LowPassFilter();
    public RampRate distanceSetpointRampRate = new RampRate();
    public PIDSourceWrapper armEncoderPIDSource = new PIDSourceWrapper();
    public WarlordsPIDController distancePID = new WarlordsPIDController();
    public WarlordsPIDController downVelocityPID = new WarlordsPIDController();
    public WarlordsPIDController upVelocityPID = new WarlordsPIDController();
    public WarlordsPIDControllerSystem cargoArmControllerSystem = new WarlordsPIDControllerSystem(this.distancePID, this.upVelocityPID, this.downVelocityPID);
    public MotorSetter motorSetter = new MotorSetter();
    public PIDSourceWrapper distanceOutputPIDSource = new PIDSourceWrapper();


    public CargoArm() {
        this.distanceSetpointRampRate.setSetpointSource(this.distanceSetpointTN);
        this.distanceSetpointRampRate.setOutputs(this.distanceSetpointRampedTN);

        this.armEncoderFilter.setSetpointSource(RobotMap.cargoArmEncoderWrapperDistance);
        this.armEncoderFilter.setOutputs(this.armEncoderTN);

        this.armEncoderPIDSource.setPidSource(() -> this.armEncoderTN.pidGet());

        this.downVelocityPID.setSetpoint(ConstantsIO.cargoArmMinVelocity);
        this.downVelocityPID.setSources(RobotMap.cargoArmEncoderWrapperRate);
        this.downVelocityPID.setOutputs(this.armMinOutputTN);
        this.downVelocityPID.setOutputRange(0.0, ConstantsIO.cargoArmIMax);

        this.upVelocityPID.setSetpoint(ConstantsIO.cargoArmMaxVelocity);
        this.upVelocityPID.setSources(RobotMap.cargoArmEncoderWrapperRate);
        this.upVelocityPID.setOutputs(this.armMaxOutputTN);
        this.upVelocityPID.setOutputRange(0.0, ConstantsIO.cargoArmIMax);

        this.distancePID.setSetpointSource(this.distanceSetpointRampedTN);
        this.distancePID.setOutputs(this.distanceOutputTN);
        this.distancePID.setSources(this.armEncoderPIDSource);
        this.distancePID.setOutputSources(this.armMaxOutputTN, this.armMinOutputTN);

        this.distanceOutputFilter.setSetpointSource(this.distanceOutputTN);
        this.distanceOutputFilter.setOutputs(this.distanceOutputFilteredTN);
        
        this.distanceOutputPIDSource.setPidSource(() -> {
            double output = this.distanceOutputFilteredTN.getOutput() * ConstantsIO.kF_cargoArmDistance;
           //spiking is checked for in default command, which is always enabled
            if (output > ConstantsIO.cargoArmIMax) {
                return ConstantsIO.cargoArmIMax;
            }
            if (output < -ConstantsIO.cargoArmIMax) {
                return -ConstantsIO.cargoArmIMax;
            }
            return output;
        });
        this.motorSetter.setSetpointSource(this.distanceOutputPIDSource);
        this.motorSetter.setOutputs(RobotMap.cargoArmCurrent);

        holdPosition = 0;
       
    }


    public void cargoArmManual(double power) {
        cargoArmControllerSystem.disable();
        this.distancePID.disable();
        this.distanceOutputTN.setOutput(0);
        RobotMap.cargoArmPercentOutput.set(power);
    }

    public void initDefaultCommand() {
       setDefaultCommand(new CargoArmWithControllers());
    }

    public void setPosition(double position) {
        this.distanceSetpointTN.setOutput(position);
        this.distancePID.enable();
        this.distancePID.setAbsoluteTolerance(1.0);
        this.enablePID(true);
    }

    public void updateConstants() {
        this.distanceSetpointRampRate.setRampRates(ConstantsIO.armDistanceSetpointUpRamp, ConstantsIO.armDistanceSetpointDownRamp);
        this.distancePID.setPID(ConstantsIO.kP_cargoArmDistance, ConstantsIO.kI_cargoArmDistance, ConstantsIO.kD_cargoArmDistance);
        this.armEncoderFilter.setFilterCoefficient(ConstantsIO.kArmEncoderFilterCoefficient);
        this.distancePID.setOutputRange(-ConstantsIO.cargoArmIMax, ConstantsIO.cargoArmIMax);
        this.distanceOutputFilter.setFilterCoefficient(ConstantsIO.kCargoArmDistanceOutputFilterCoefficient);
        this.downVelocityPID.setPID(ConstantsIO.kP_cargoArmDownVelocity, ConstantsIO.kI_cargoArmDownVelocity, ConstantsIO.kD_cargoArmDownVelocity);
        this.upVelocityPID.setPID(ConstantsIO.kP_cargoArmUpVelocity, ConstantsIO.kI_cargoArmUpVelocity, ConstantsIO.kD_cargoArmUpVelocity);
    }

    public void enablePID(boolean enabled) {
        if (enabled) {
            this.motorSetter.enable();
            this.cargoArmControllerSystem.enable();
            this.distanceSetpointRampRate.enable();
            this.armEncoderFilter.enable();
            this.distanceOutputFilter.enable();
        } else {
            this.cargoArmControllerSystem.disable();
            RobotMap.cargoArmCurrent.set(0);
        }
    }


}