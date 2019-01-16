package org.usfirst.frc.team2485.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.ThresholdHandler;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;
import org.usfirst.frc.team2485.util.WarlordsPIDController;
import org.usfirst.frc.team2485.util.RampRate;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.MotorSetter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import org.usfirst.frc.team2485.util.AutoPath.Pair;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.FastMath;

public class DriveTrain extends Subsystem {
	private static final double THETA_CAMERA = 60;
	private static final double CAMERA_PIXEL_WIDTH = 320;

	private static final double MAX_VEL = 10; //change
	private static final double MAX_ANGVEL = 10; //change
	private static final double MAX_ANG = Math.PI; 
	private static final double MAX_CURR = 10; 

	public WarlordsPIDController distancePID;
	private WarlordsPIDController velocityPID;
	public WarlordsPIDController anglePID;
	private WarlordsPIDController angVelPID;

	private RampRate velocityRampRate;

	private TransferNode distanceTN;
	private TransferNode velocitySetpointTN;
	private TransferNode velocityTN;
	private TransferNode angVelTN;
	private TransferNode angleTN;

	//public PIDSourceWrapper kp_distancePIDSource;
	private PIDSourceWrapper distancePIDSource;
	private PIDSourceWrapper velocityPIDSource;
	private PIDSourceWrapper anglePIDSource;
	private PIDSourceWrapper angVelPIDSource;
	private PIDSourceWrapper minVelocityORSource;
	private PIDSourceWrapper maxVelocityORSource;
	private PIDSourceWrapper minAngleORSource;
	private PIDSourceWrapper maxAngleORSource;

	private PIDSourceWrapper leftCurrentPIDSource;
	private PIDSourceWrapper rightCurrentPIDSource;

	private MotorSetter leftMotorSetter;
	private MotorSetter rightMotorSetter;




	public DriveTrain(){
		distancePID = new WarlordsPIDController();
		velocityPID = new WarlordsPIDController();
		anglePID = new WarlordsPIDController();
		angVelPID = new WarlordsPIDController();

		velocityRampRate = new RampRate();
		
		distanceTN = new TransferNode(0);
		velocitySetpointTN = new TransferNode(0);
		velocityTN = new TransferNode(0);
		angVelTN = new TransferNode(0);
		angleTN = new TransferNode(0);

		//kp_distancePIDSource = new PIDSourceWrapper();
		distancePIDSource = new PIDSourceWrapper();
		velocityPIDSource = new PIDSourceWrapper();
		anglePIDSource = new PIDSourceWrapper();
		angVelPIDSource = new PIDSourceWrapper();


		minVelocityORSource = new PIDSourceWrapper();
		maxVelocityORSource = new PIDSourceWrapper();
		minAngleORSource = new PIDSourceWrapper();
		maxAngleORSource = new PIDSourceWrapper();

		leftCurrentPIDSource = new PIDSourceWrapper();
		rightCurrentPIDSource = new PIDSourceWrapper();

		leftMotorSetter = new MotorSetter();
		rightMotorSetter = new MotorSetter();

		//distancePID.setSetpointSource()
		distancePIDSource.setPidSource(() -> {
			return (RobotMap.driveLeftEncoderWrapperDistance.pidGet()
					+ RobotMap.driveRightEncoderWrapperDistance.pidGet()) / 2;
		});

		distancePID.setSources(distancePIDSource);
		distancePID.setOutputRange(-MAX_VEL, MAX_VEL);
		distancePID.setOutputs(distanceTN);

		velocityRampRate.setSetpointSource(distanceTN);
		velocityRampRate.setRampRates(ConstantsIO.kRamp_VelocityRate, ConstantsIO.kRamp_VelocityRate);
		velocityRampRate.setOutputs(velocitySetpointTN);

		velocityPIDSource.setPidSource(() -> {
			return (RobotMap.driveLeftEncoderWrapperRate.pidGet() + RobotMap.driveRightEncoderWrapperRate.pidGet()) / 2;
		});

		velocityPID.setSources(velocityPIDSource);
		velocityPID.setSetpointSource(velocitySetpointTN);
		velocityPID.setOutputs(velocityTN);
		velocityPID.setOutputRange(0, MAX_CURR);
		// output range of velocity PID is negative to positive current max


		anglePIDSource.setPidSource(()-> {
			return RobotMap.gyroAngleWrapper.pidGet();
		});


		anglePID.setSources(anglePIDSource);

		anglePID.setContinuous(true);
		anglePID.setInputRange(-MAX_ANG, MAX_ANG);
		anglePID.setOutputRange(-MAX_ANGVEL, MAX_ANGVEL);

		anglePID.setOutputs(angleTN);



		angVelPIDSource.setPidSource(()-> {
			return RobotMap.gyroRateWrapper.pidGet();
		});

		angVelPID.setSources(angVelPIDSource);

		angVelPID.setSetpointSource(angleTN);

		angVelPID.setOutputRange(0, MAX_CURR);

		angVelPID.setOutputs(angVelTN);

		leftCurrentPIDSource.setPidSource(() ->{
			return velocityTN.pidGet() + angVelTN.pidGet();
		});

		rightCurrentPIDSource.setPidSource(() ->{
			return velocityTN.pidGet() - angVelTN.pidGet();
		});

		leftMotorSetter.setSources(leftCurrentPIDSource);

		rightMotorSetter.setSources(rightCurrentPIDSource);

		leftMotorSetter.setOutputs(RobotMap.driveLeft);

		rightMotorSetter.setOutputs(RobotMap.driveRight);










		
		
	}

	public void updateConstants(){
		
	}
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    	setDefaultCommand(new DriveWithControllers());
	}
	
	
    public void enablePID(boolean PID){
		if(PID){
		velocityPID.enable();
		anglePID.enable();
		distancePID.enable();
		velocityRampRate.enable();
		angVelPID.enable();
		} else {
			velocityPID.disable();
		anglePID.disable();
		distancePID.disable();
		velocityRampRate.disable();
		angVelPID.disable();
		}
	}
    public void simpleDrive(double x, double y, boolean quickTurn) {
		enablePID(false);
    	
    	double steering = ThresholdHandler.deadbandAndScale(x, 0.2, 0, 1);
		
		double throttle = ThresholdHandler.deadbandAndScale(y, 0.2, 0, 1);
		
		double left;
		
		double right;
		
		if (quickTurn) {
			left=steering;
			right=-steering;
		} else {
			left=throttle+steering*throttle;
			right=throttle-steering*throttle;
			//scale the speeds down so they are in range (-1,1)
			if (Math.abs(left)>1) { 
				right/=Math.abs(left);
				left/=Math.abs(left);
			}
			
			if (Math.abs(right)>1) {
				left/=Math.abs(right);
				right/=Math.abs(right);
			}
		}
		
		
		RobotMap.driveLeft.set(left);
		
		RobotMap.driveRight.set(right);
		
	}
    
    public void driveAtSpeed(double speed) {
    	RobotMap.driveLeft.set(speed);
    	RobotMap.driveRight.set(speed);
    }
    
    public void stop() {
    	RobotMap.driveLeft.set(0);
    	RobotMap.driveRight.set(0);
	}
	
	public double getSurfaceAngle() {
		double phi = RobotMap.gyroAngleWrapper.pidGet(); //gyro angle
		double theta = 0; //the surface angle 

		if (phi < 0) {
			phi = 2*Math.PI - Math.abs(phi);
		}

		if (phi >= Math.toRadians(345.625) || phi < Math.toRadians(14.375)) {
			theta = Math.toRadians(90); 
		} else if (phi >= Math.toRadians(14.375) || phi < Math.toRadians(59.375)) {
			theta = Math.toRadians(118.75);
		} else if (phi >= Math.toRadians(59.375) || phi < Math.toRadians(120.625)) {
			theta = Math.toRadians(0);
		} else if (phi >= Math.toRadians(120.625) || phi < Math.toRadians(180)) {
			theta = Math.toRadians(61.25);
		} else if (phi >= Math.toRadians(180) || phi < Math.toRadians(239.375)) {
			theta = Math.toRadians(298.75);
		} else if (phi >= Math.toRadians(239.375) || phi < Math.toRadians(300.625)) {
			theta = Math.toRadians(0);
		} else if (phi >= Math.toRadians(300.625) || phi < Math.toRadians(345.625)) {
			theta = Math.toRadians(241.25);
		} 

		return theta;
	}

	public Pair getAutoAlignEndpoint() {
		double lidarDist = RobotMap.lidar.getDistance();
		double thetaSurface = getSurfaceAngle();
		double phi = RobotMap.gyroAngleWrapper.pidGet();
		double cx1 = 0; //change
		double cx2 = 0; //change
		double thetaCamera = THETA_CAMERA; //angle of end of field of view to plane parallel to robot
		double thetaFieldOfView = Math.PI - thetaCamera*2; //total field of view angle
		double fovWidth = FastMath.sin(thetaFieldOfView/2)*lidarDist*2; //width in inches of total field of view (assuming lidar dist as height)
		double thetaRobotToSurface = thetaSurface - phi; //angle between plane of surface and plane parallel to front of robot
		double cx = (cx1+cx2)/2; //distance to alignment line in pixels (x-axis)
		double Px = cx*fovWidth/CAMERA_PIXEL_WIDTH; //distance to alignment line in inches (x-axis)
		double cy = FastMath.tan(thetaRobotToSurface)*Px; 
		double Py = cy + lidarDist;
		



		return new Pair(Px, Py);
 
	}
}

