package org.usfirst.frc.team2485.robot.subsystems;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.MotorSetter;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveTrain extends Subsystem {
    public static final double MAX_VELOCITY = 70;

    public WarlordsPIDController distancePID;
    public WarlordsPIDController velocityPID;
    public WarlordsPIDController anglePID;
    public WarlordsPIDController angVelPID;

    public TransferNode distanceSetpointTN;
    public TransferNode angleSetpointTN;
    public TransferNode distanceOutputTN;
    public TransferNode velocityOutputTN;
    public TransferNode angleOutputTN;
    public TransferNode angVelOutputTN;

    public PIDSourceWrapper distancePIDSource;
    public PIDSourceWrapper velocityPIDSource;
    public PIDSourceWrapper leftCurrentPIDSource;
    public PIDSourceWrapper rightCurrentPIDSource;

    public MotorSetter leftMotorSetter;
    public MotorSetter rightMotorSetter;

    public DriveTrain() {

        distancePID = new WarlordsPIDController();
        velocityPID = new WarlordsPIDController();
        anglePID = new WarlordsPIDController();
        angVelPID = new WarlordsPIDController();

        distanceSetpointTN = new TransferNode(0);
        angleSetpointTN = new TransferNode(0);
        distanceOutputTN = new TransferNode(0); 
        velocityOutputTN = new TransferNode(0);
        angleOutputTN = new TransferNode(0);
        angVelOutputTN = new TransferNode(0);

        distancePIDSource = new PIDSourceWrapper();
        velocityPIDSource = new PIDSourceWrapper();
        leftCurrentPIDSource = new PIDSourceWrapper();
        rightCurrentPIDSource = new PIDSourceWrapper();

        leftMotorSetter = new MotorSetter();
        rightMotorSetter = new MotorSetter();

        distancePIDSource.setPidSource(() -> {
            return (RobotMap.driveLeftEncoderWrapperDistance.pidGet() + RobotMap.driveRightEncoderWrapperDistance.pidGet())/2;
        });

        distancePID.setSetpointSource(distanceSetpointTN);
        distancePID.setSources(distancePIDSource);
        distancePID.setOutputRange(-MAX_VELOCITY, MAX_VELOCITY);
        distancePID.setOutputs(distanceOutputTN);

        velocityPIDSource.setPidSource(() -> {
            return (RobotMap.driveLeftEncoderWrapperRate.pidGet() + RobotMap.driveRightEncoderWrapperRate.pidGet())/2;
        });

        velocityPID.setSetpointSource(distanceOutputTN);
        velocityPID.setSources(velocityPIDSource);
        velocityPID.setOutputRange(-ConstantsIO.driveTrainIMax, ConstantsIO.driveTrainIMax);
        velocityPID.setOutputs(velocityOutputTN);

        anglePID.setSetpointSource(angleSetpointTN);
        anglePID.setSources(RobotMap.gyroAngleWrapper);
        anglePID.setOutputRange(-MAX_VELOCITY, MAX_VELOCITY);
        anglePID.setOutputs(angleOutputTN);

        angVelPID.setSetpointSource(angleOutputTN);
        angVelPID.setSources(RobotMap.gyroRateWrapper);
        angVelPID.setOutputRange(-ConstantsIO.driveTrainIMax, ConstantsIO.driveTrainIMax);
        angVelPID.setOutputs(angVelOutputTN);


        leftCurrentPIDSource.setPidSource(() -> {
           return velocityOutputTN.pidGet() + angVelOutputTN.pidGet(); 
        });

        rightCurrentPIDSource.setPidSource(() -> {
            return velocityOutputTN.pidGet() - angVelOutputTN.pidGet();
        });

        leftMotorSetter.setSetpointSource(leftCurrentPIDSource);
        leftMotorSetter.setOutputs(RobotMap.driveLeftCurrent);
        rightMotorSetter.setSetpointSource(rightCurrentPIDSource);
        rightMotorSetter.setOutputs(RobotMap.driveRightCurrent);


    }

    public void WarlordsDrive(double throttle, double steering, boolean quickTurn) {
        if(quickTurn) {
            RobotMap.driveLeftPercentOutput.set(steering);
            RobotMap.driveRightPercentOutput.set(-steering);
        } else {
            double left = throttle + Math.abs(throttle)*steering;
            double right = throttle - Math.abs(throttle)*steering;

            if(Math.abs(left) > 1) {
                right /= Math.abs(left);
                left /= Math.abs(left);
            } else if (Math.abs(right) > 1) {
                left /= Math.abs(right);
                right /= Math.abs(right);
            }

            RobotMap.driveLeftPercentOutput.set(left);
            RobotMap.driveRightPercentOutput.set(right);
        }
    }

    public void initDefaultCommand() {
        setDefaultCommand(new DriveWithControllers());
    }

    public void updateConstants() {
        velocityPID.setPID(ConstantsIO.kP_DriveVelocity, ConstantsIO.kI_DriveVelocity, ConstantsIO.kD_DriveVelocity, ConstantsIO.kF_DriveVelocity);
        anglePID.setPID(ConstantsIO.kP_DriveAngle, ConstantsIO.kI_DriveAngle, ConstantsIO.kD_DriveAngle);
        angVelPID.setPID(ConstantsIO.kP_DriveAngVel, ConstantsIO.kI_DriveAngVel, ConstantsIO.kD_DriveAngVel, ConstantsIO.kF_DriveAngVel);
    }

    public boolean driveTo(double distance, double maxSpeed, double angle, double curvature, double toleranceDist, double toleranceAngle) {
		anglePID.enable();
		distancePID.enable();
		velocityPID.enable();
		angVelPID.enable();
		//velocityRampRate.enable();
		leftMotorSetter.enable();
		rightMotorSetter.enable();
		angleSetpointTN.setOutput(angle);
		distanceSetpointTN.setOutput(distance);
		distancePID.setAbsoluteTolerance(toleranceDist);
		anglePID.setAbsoluteTolerance(toleranceAngle);
		
		distancePID.setOutputRange(-maxSpeed, maxSpeed);

		return distancePID.isOnTarget() && anglePID.isOnTarget();
	}



    public void enablePID(boolean enabled) {
        if (enabled) {
            distancePID.enable();
            velocityPID.enable();
            anglePID.enable();
            angVelPID.enable();
            leftMotorSetter.enable();
            rightMotorSetter.enable();
        } else {
            distancePID.disable();
            velocityPID.disable();
            anglePID.disable();
            angVelPID.disable();
            leftMotorSetter.disable();
            rightMotorSetter.disable();
        }
    }
}