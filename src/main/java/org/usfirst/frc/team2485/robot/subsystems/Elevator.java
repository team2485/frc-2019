package org.usfirst.frc.team2485.robot.subsystems;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.ElevatorWithControllers;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.EncoderWrapperRateAndDistance;
import org.usfirst.frc.team2485.util.LowPassFilter;
import org.usfirst.frc.team2485.util.MotorSetter;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.RampRate;
import org.usfirst.frc.team2485.util.SpeedControllerWrapper;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;
import org.usfirst.frc.team2485.util.WarlordsPIDControllerSystem;
import org.usfirst.frc.team2485.util.WarlordsPIDSource;

public class Elevator extends Subsystem {
    public static boolean enableFailsafe = false;
    public ElevatorLevel lastLevel;
    public TransferNode distanceSetpointTN = new TransferNode(0.0);
    public TransferNode distanceSetpointRampedTN = new TransferNode(0.0);
    public TransferNode elevatorEncoderTN = new TransferNode(0.0);
    public TransferNode failsafeTN = new TransferNode(0.0);
    public TransferNode distanceOutputTN = new TransferNode(0.0);
    public TransferNode distanceOutputFilteredTN = new TransferNode(0.0);
    public TransferNode elevatorMaxOutputTN = new TransferNode(0.0);
    public TransferNode elevatorMinOutputTN = new TransferNode(0.0);
    public LowPassFilter elevatorEncoderFilter = new LowPassFilter();
    public LowPassFilter distanceOutputFilter = new LowPassFilter();
    public RampRate distanceSetpointRampRate = new RampRate();
    public PIDSourceWrapper elevatorEncoderPIDSource = new PIDSourceWrapper();
    public WarlordsPIDController distancePID = new WarlordsPIDController();
    public WarlordsPIDController downVelocityPID = new WarlordsPIDController();
    public WarlordsPIDController upVelocityPID = new WarlordsPIDController();
    public WarlordsPIDControllerSystem elevatorControllerSystem = new WarlordsPIDControllerSystem(this.distancePID, this.upVelocityPID, this.downVelocityPID);
    public MotorSetter motorSetter = new MotorSetter();
    public PIDSourceWrapper distanceOutputPIDSource = new PIDSourceWrapper();

    public Elevator() {

        this.distanceSetpointRampRate.setSetpointSource(this.distanceSetpointTN);
        this.distanceSetpointRampRate.setOutputs(this.distanceSetpointRampedTN);

        this.elevatorEncoderFilter.setSetpointSource(RobotMap.elevatorEncoderWrapperDistance);
        this.elevatorEncoderFilter.setOutputs(this.elevatorEncoderTN);

        this.elevatorEncoderPIDSource.setPidSource(() -> this.elevatorEncoderTN.pidGet());

        this.downVelocityPID.setSetpoint(ConstantsIO.elevatorMinVelocity);
        this.downVelocityPID.setSources(RobotMap.elevatorEncoderWrapperRate);
        this.downVelocityPID.setOutputs(this.elevatorMinOutputTN);
        this.downVelocityPID.setOutputRange(0.0, ConstantsIO.elevatorIMax);

        this.upVelocityPID.setSetpoint(ConstantsIO.elevatorMaxVelocity);
        this.upVelocityPID.setSources(RobotMap.elevatorEncoderWrapperRate);
        this.upVelocityPID.setOutputs(this.elevatorMaxOutputTN);
        this.upVelocityPID.setOutputRange(0.0, ConstantsIO.elevatorIMax);

        this.distancePID.setSetpointSource(this.distanceSetpointRampedTN);
        this.distancePID.setOutputs(this.distanceOutputTN); 
        this.distancePID.setSources(this.elevatorEncoderPIDSource);
        this.distancePID.setOutputSources(this.elevatorMaxOutputTN, this.elevatorMinOutputTN);

        this.distanceOutputFilter.setSetpointSource(this.distanceOutputTN);
        this.distanceOutputFilter.setOutputs(this.distanceOutputFilteredTN);
        
        this.distanceOutputPIDSource.setPidSource(() -> {
            double output = this.distanceOutputFilteredTN.getOutput() * ConstantsIO.kF_elevatorDistance;
            if (enableFailsafe) {
                return 0.0;
            }
            if (output > ConstantsIO.elevatorIMax) {
                return ConstantsIO.elevatorIMax;
            }
            if (output < -ConstantsIO.elevatorIMax) {
                return -ConstantsIO.elevatorIMax;
            }
            return output;
        });
        this.motorSetter.setSetpointSource(this.distanceOutputPIDSource);
        this.motorSetter.setOutputs(RobotMap.elevatorCurrent);
        this.lastLevel = ElevatorLevel.FLOOR;
    }

    public void elevatorManual(double power) {
        elevatorControllerSystem.disable();
        this.distancePID.disable();
        this.distanceOutputTN.setOutput(0);
        RobotMap.elevatorPercentOutput.set(power);
    }

    public void setLevel(ElevatorLevel level) {
        this.setPosition(level.getPosition());
        this.lastLevel = level;
    }

    public void setPosition(double position) {
        this.distanceSetpointTN.setOutput(position);
        this.distancePID.enable();
        this.distancePID.setAbsoluteTolerance(1.0);
        this.enablePID(true);
    }

    @Override
    public void initDefaultCommand() {
        this.setDefaultCommand(new ElevatorWithControllers());
    }

    public void enablePID(boolean enable) {
        if (enable) {
           // this.elevatorControllerSystem.enable();
           this.downVelocityPID.enable();
           this.upVelocityPID.enable();
           this.distancePID.enable();
            this.distanceSetpointRampRate.enable();
            this.motorSetter.enable();
            this.elevatorEncoderFilter.enable();
            this.distanceOutputFilter.enable();
        } else {
            //this.elevatorControllerSystem.disable();
            this.downVelocityPID.disable();
            this.upVelocityPID.disable();
            this.distancePID.disable();
            RobotMap.elevator.distanceOutputTN.setOutput(0);
        }
    }

    public void updateConstants() {
        this.distanceSetpointRampRate.setRampRates(ConstantsIO.elevatorDistanceSetpointUpRamp, ConstantsIO.elevatorDistanceSetpointDownRamp);
        this.distancePID.setPID(ConstantsIO.kP_elevatorDistance, ConstantsIO.kI_elevatorDistance, ConstantsIO.kD_elevatorDistance);
        this.elevatorEncoderFilter.setFilterCoefficient(ConstantsIO.kElevatorEncoderFilterCoefficient);
        this.distancePID.setOutputRange(-ConstantsIO.elevatorIMax, ConstantsIO.elevatorIMax);
        this.distanceOutputFilter.setFilterCoefficient(ConstantsIO.kElevatorDistanceOutputFilterCoefficient);
        this.downVelocityPID.setPID(ConstantsIO.kP_elevatorDownVelocity, ConstantsIO.kI_elevatorDownVelocity, ConstantsIO.kD_elevatorDownVelocity);
        this.upVelocityPID.setPID(ConstantsIO.kP_elevatorUpVelocity, ConstantsIO.kI_elevatorUpVelocity, ConstantsIO.kD_elevatorUpVelocity);
    }

    public static enum ElevatorLevel {
        FLOOR,
        ROCKET_LEVEL_ONE,
        ROCKET_LEVEL_TWO,
        ROCKET_LEVEL_THREE,
        HATCH_LIFTING,
        HATCH_INTAKE_FLOOR;
        

        public double getPosition() {
            switch (this) {
                case FLOOR: {
                    return 0.0;
                }
                case ROCKET_LEVEL_ONE: {
                    return 5.5;
                }
                case ROCKET_LEVEL_TWO: {
                    return 30;
                }
                case ROCKET_LEVEL_THREE: {
                    return 58.1;
                }
                case HATCH_LIFTING: {
                    return 12.0;
                }
                case HATCH_INTAKE_FLOOR: {
                    return 2.0;
                }
            }
            return 0.0;
        }
    }
}