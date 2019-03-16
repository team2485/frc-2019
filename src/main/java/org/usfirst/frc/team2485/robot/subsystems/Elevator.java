package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.ElevatorWithControllers;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.LowPassFilter;
import org.usfirst.frc.team2485.util.MotorSetter;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.RampRate;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import edu.wpi.first.wpilibj.command.Subsystem;



public class Elevator extends Subsystem {
    public static boolean enableFailsafe = false;

    public enum ElevatorLevel {
        FLOOR, ROCKET_LEVEL_ONE, ROCKET_LEVEL_TWO, ROCKET_LEVEL_THREE, HATCH_LIFTING, HATCH_INTAKE_FLOOR;
        public double getPosition() {
            switch(this) {
                case FLOOR:
                    return 4.5;
                case ROCKET_LEVEL_ONE:
                    return 6;
                case ROCKET_LEVEL_TWO:
                    return 33.5;
                case ROCKET_LEVEL_THREE:
                    return 61;
                case HATCH_LIFTING:
                    return 12;
                case HATCH_INTAKE_FLOOR:
                    return 0;
                default:
                    return 0;
            }
        }
    }

    public ElevatorLevel lastLevel;

    public TransferNode distanceSetpointTN;
    public TransferNode distanceSetpointRampedTN;
    public TransferNode elevatorEncoderTN;
    public TransferNode failsafeTN;
    public TransferNode distanceOutputTN;
    public TransferNode distanceOutputFilteredTN;

    public LowPassFilter elevatorEncoderFilter;
    public LowPassFilter distanceOutputFilter;

    public RampRate distanceSetpointRampRate;

    public PIDSourceWrapper elevatorEncoderPIDSource;

    public WarlordsPIDController distancePID;

    public MotorSetter motorSetter;
    public PIDSourceWrapper distanceOutputPIDSource;

    


    public Elevator() {
        distanceSetpointTN = new TransferNode(0);
        distanceSetpointRampedTN = new TransferNode(0);
        elevatorEncoderTN = new TransferNode(0);
        failsafeTN = new TransferNode(0);
        distanceOutputTN = new TransferNode(0);
        distanceOutputFilteredTN = new TransferNode(0);

        distanceSetpointRampRate = new RampRate();

        distancePID = new WarlordsPIDController();

        elevatorEncoderPIDSource = new PIDSourceWrapper();
        distanceOutputPIDSource = new PIDSourceWrapper();

        elevatorEncoderFilter = new LowPassFilter();
        distanceOutputFilter = new LowPassFilter();

        motorSetter = new MotorSetter();

        distanceSetpointRampRate.setSetpointSource(distanceSetpointTN);
        distanceSetpointRampRate.setOutputs(distanceSetpointRampedTN);

       
        elevatorEncoderFilter.setSetpointSource(RobotMap.elevatorEncoderWrapperDistance);
        elevatorEncoderFilter.setOutputs(elevatorEncoderTN);

        elevatorEncoderPIDSource.setPidSource(() -> {
            return elevatorEncoderTN.pidGet();
        });


        distancePID.setSetpointSource(distanceSetpointRampedTN);
        distancePID.setOutputs(distanceOutputTN);
        distancePID.setSources(elevatorEncoderPIDSource);
        distancePID.setOutputRange(-ConstantsIO.elevatorIMax, ConstantsIO.elevatorIMax);

        distanceOutputFilter.setSetpointSource(distanceOutputTN);
        distanceOutputFilter.setOutputs(distanceOutputFilteredTN);


        distanceOutputPIDSource.setPidSource(() -> {
            double output = distanceOutputFilteredTN.getOutput() * ConstantsIO.kF_elevatorDistance;
            if(enableFailsafe) {
                return 0;
            } else { 
                if(output > ConstantsIO.elevatorIMax){
                    return ConstantsIO.elevatorIMax;
                } else if (output < -ConstantsIO.elevatorIMax){
                    return -ConstantsIO.elevatorIMax;
                } else { 
                    return output;
                }
            } 
        });

        
        motorSetter.setSetpointSource(distanceOutputPIDSource);
        motorSetter.setOutputs(RobotMap.elevatorCurrent);


        lastLevel = ElevatorLevel.FLOOR;
    }

    public void elevatorManual(double power) {
       RobotMap.elevatorPercentOutput.set(power);
    }

    public void setLevel(ElevatorLevel level) {
        this.setPosition(level.getPosition());
        this.lastLevel = level;
    }

    public void setPosition(double position) {
        this.distanceSetpointTN.setOutput(position);
        this.distancePID.enable();
        this.distancePID.setAbsoluteTolerance(1);
        this.enablePID(true);
    }

    public void initDefaultCommand() {
        setDefaultCommand(new ElevatorWithControllers());
    }

    public void enablePID(boolean enable) {
        if (enable) {
            distancePID.enable();
            distanceSetpointRampRate.enable();
            motorSetter.enable();
            elevatorEncoderFilter.enable();
            distanceOutputFilter.enable();
        } else {
            // distanceSetpointTN.setOutput(0);
            // distanceSetpointRampedTN.setOutput(0);
            distancePID.disable();
        }
    }

    public void updateConstants() {
        distanceSetpointRampRate.setRampRates(ConstantsIO.elevatorDistanceSetpointUpRamp, ConstantsIO.elevatorDistanceSetpointDownRamp);
        distancePID.setPID(ConstantsIO.kP_elevatorDistance, ConstantsIO.kI_elevatorDistance, ConstantsIO.kD_elevatorDistance);
        elevatorEncoderFilter.setFilterCoefficient(ConstantsIO.kElevatorEncoderFilterCoefficient);
        distancePID.setOutputRange(-ConstantsIO.elevatorIMax, ConstantsIO.elevatorIMax);
        distanceOutputFilter.setFilterCoefficient(ConstantsIO.kElevatorDistanceOutputFilterCoefficient);
    }
}