package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.ElevatorWithControllers;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.MotorSetter;
import org.usfirst.frc.team2485.util.RampRate;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import edu.wpi.first.wpilibj.command.Subsystem;



public class Elevator extends Subsystem {
    public final double HOLDING_CURRENT = 5;

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

    public RampRate distanceSetpointRampRate;

    public WarlordsPIDController distancePID;


    public Elevator() {
        distanceSetpointTN = new TransferNode(0);
        distanceSetpointRampedTN = new TransferNode(0);

        distanceSetpointRampRate = new RampRate();

        distancePID = new WarlordsPIDController();

        distanceSetpointRampRate.setSetpointSource(distanceSetpointTN);
        distanceSetpointRampRate.setOutputs(distanceSetpointRampedTN);

        distancePID.setSetpointSource(distanceSetpointRampedTN);
        distancePID.setOutputs(RobotMap.elevatorCurrent);
        distancePID.setSources(RobotMap.elevatorEncoderWrapperDistance);
        distancePID.setOutputRange(-ConstantsIO.elevatorIMax, ConstantsIO.elevatorIMax);

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
        } else {
            // distanceSetpointTN.setOutput(0);
            // distanceSetpointRampedTN.setOutput(0);
            distancePID.disable();
        }
    }

    public void updateConstants() {
        distanceSetpointRampRate.setRampRates(ConstantsIO.elevatorDistanceSetpointUpRamp, ConstantsIO.elevatorDistanceSetpointDownRamp);
        distancePID.setPID(ConstantsIO.kP_elevatorDistance, ConstantsIO.kI_elevatorDistance, ConstantsIO.kD_elevatorDistance);
        
        distancePID.setOutputRange(-ConstantsIO.elevatorIMax, ConstantsIO.elevatorIMax);
    }
}