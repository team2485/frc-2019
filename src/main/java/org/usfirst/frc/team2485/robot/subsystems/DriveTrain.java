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
	private static final double MAX_CURR = 30; 

	public WarlordsPIDController distancePID;
	public WarlordsPIDController velocityPID;
	public WarlordsPIDController anglePID;
	public WarlordsPIDController angVelPID;

	private RampRate velocityRampRate;

	public TransferNode distanceTN;
	public TransferNode velocitySetpointTN;
	public TransferNode velocityTN;
	public TransferNode angVelTN;
	public TransferNode angleTN;

	//public PIDSourceWrapper kp_distancePIDSource;
	private PIDSourceWrapper distancePIDSource;
	public PIDSourceWrapper velocityPIDSource;
	private PIDSourceWrapper anglePIDSource;
	private PIDSourceWrapper angVelPIDSource;
	private PIDSourceWrapper minVelocityORSource;
	private PIDSourceWrapper maxVelocityORSource;
	private PIDSourceWrapper minAngleORSource;
	private PIDSourceWrapper maxAngleORSource;

	public PIDSourceWrapper leftCurrentPIDSource;
	public PIDSourceWrapper rightCurrentPIDSource;

	public MotorSetter leftMotorSetter;
	public MotorSetter rightMotorSetter;




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

		updateConstants();

		distancePIDSource.setPidSource(() -> {
			return ((RobotMap.driveLeftEncoderWrapperDistance.pidGet()
					+ RobotMap.driveRightEncoderWrapperDistance.pidGet()) / 2);
		});

		distancePID.setSources(distancePIDSource);
		distancePID.setOutputRange(-MAX_VEL, MAX_VEL);
		distancePID.setOutputs(distanceTN);

		velocityRampRate.setSetpointSource(distanceTN);
		velocityRampRate.setOutputs(velocitySetpointTN);

		velocityPIDSource.setPidSource(() -> {
			return ((((RobotMap.driveLeftEncoderWrapperRate.pidGet() + RobotMap.driveRightEncoderWrapperRate.pidGet()) / 2)));
		});

		velocityPID.setSources(velocityPIDSource);
		velocityPID.setSetpointSource(velocitySetpointTN);
		velocityPID.setOutputs(velocityTN);
		velocityPID.setOutputRange(-MAX_CURR, MAX_CURR);
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
		angVelPID.setOutputRange(-MAX_CURR, MAX_CURR);
		angVelPID.setOutputs(angVelTN);

		leftCurrentPIDSource.setPidSource(() ->{
			return velocityTN.pidGet() - angVelTN.pidGet();
		});

		rightCurrentPIDSource.setPidSource(() ->{
			return velocityTN.pidGet() + angVelTN.pidGet();
		});

		leftMotorSetter.setSetpointSource(leftCurrentPIDSource);
		rightMotorSetter.setSetpointSource(rightCurrentPIDSource);
		leftMotorSetter.setOutputs(RobotMap.driveLeftPWM);
		rightMotorSetter.setOutputs(RobotMap.driveRightPWM);

		
	}

	public void updateConstants() {
		
		distancePID.setPID(ConstantsIO.kPMax_Distance, 0, 0);
		velocityPID.setPID(ConstantsIO.kP_DriveVelocity, ConstantsIO.kI_DriveVelocity, 0, ConstantsIO.kF_DriveVelocity);
		velocityRampRate.setRampRates(ConstantsIO.kUpRamp_Velocity, ConstantsIO.kDownRamp_Velocity);
		anglePID.setPID(ConstantsIO.kP_DriveAngle, ConstantsIO.kI_DriveAngle, ConstantsIO.kD_DriveAngle);
		angVelPID.setPID(ConstantsIO.kP_AngVelTeleop, ConstantsIO.kI_AngVelTeleop, 0, ConstantsIO.kF_AngVelTeleop);
		
	}
	
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
		// setDefaultCommand(new DriveWithControllers());
	}
	
	
    public void enablePID(boolean PID){
		if(PID){
			velocityPID.enable();
			anglePID.enable();
			distancePID.enable();
			velocityRampRate.enable();
			angVelPID.enable();
			leftMotorSetter.enable();
			rightMotorSetter.enable();
		} else {
			velocityPID.disable();
			anglePID.disable();
			distancePID.disable();
			velocityRampRate.disable();
			angVelPID.disable();
			leftMotorSetter.disable();
			rightMotorSetter.disable();
		}
	}
    public void simpleDrive(double y, double x, boolean quickTurn) {
		enablePID(false);
    	
    	double steering = ThresholdHandler.deadbandAndScale(x, 0.2, 0, 1);
		
		double throttle = ThresholdHandler.deadbandAndScale(y, 0.2, 0, 1);

		// System.out.println("Steering: " + steering);
		// System.out.println("Throttle: " + throttle);
		
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
		
		// System.out.println("Left: " + left);
		// System.out.println("Right: " + right);

		RobotMap.driveLeftPWM.set(left);
		RobotMap.driveRightPWM.set(right);


		
	}
    
   
	
	public static double getThetaSurface() {
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

	public static Pair getAutoAlignEndpoint() {
		double lidarDist = RobotMap.lidar.getDistance();
		double thetaSurface = getThetaSurface();
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

	public boolean driveTo(double distance, double maxSpeed, double angle, double curvature, double toleranceDist, double toleranceAngle) {
		anglePID.enable();
		distancePID.enable();
		velocityPID.enable();
		angVelPID.enable();
		velocityRampRate.enable();
		leftMotorSetter.enable();
		rightMotorSetter.enable();
		angleTN.setOutput(angle);
		distancePID.setSetpoint(distance);
		distancePID.setAbsoluteTolerance(toleranceDist);
		anglePID.setAbsoluteTolerance(toleranceAngle);
		
		distancePID.setOutputRange(-maxSpeed, maxSpeed);

		return distancePID.isOnTarget() && anglePID.isOnTarget();
	}

	public static void setHighLowCurrent(double highCurrentLeft, double lowCurrentLeft, double highCurrentRight, double lowCurrentRight, int period){
		RobotMap.driveLeftCurrent.set(highCurrentLeft);
		RobotMap.driveRightCurrent.set(highCurrentRight);
		try {
			Thread.sleep(period);
		} catch (Exception e) {
			//TODO: handle exception
		}
		RobotMap.driveLeftCurrent.set(lowCurrentLeft);
		RobotMap.driveRightCurrent.set(lowCurrentRight);
		try {
			Thread.sleep(period);
		} catch (Exception e) {
			//TODO: handle exception
		}
	}

	public void setVelocities(double linearVelocity, double angularVelocity) {
		distancePID.disable();
		anglePID.disable();
		leftMotorSetter.enable();
		rightMotorSetter.enable();
		velocityPID.enable();
		angVelPID.enable();
		velocityRampRate.enable();
		


		velocityPID.setSetpointSource(null);
		velocityPID.setSetpoint(linearVelocity);
		angVelPID.setSetpointSource(null);
		angVelPID.setSetpoint(angularVelocity);

	}

	public void setAngle(double angle) {
		velocityPID.disable();
		anglePID.enable();
		distancePID.disable();
		velocityRampRate.disable();
		angVelPID.enable();
		leftMotorSetter.enable();
		rightMotorSetter.enable();
		anglePID.setSetpointSource(null);
		anglePID.setSetpoint(angle);

	}

	public static Pair[] generateControlPoints() {
		Pair endpoint = getAutoAlignEndpoint();
		Pair startpoint = new Pair(0,0);
		double thetaAlignmentLine = (Math.PI/2) - getThetaSurface();
		double Pc1y = (endpoint.getY() - startpoint.getY())*ConstantsIO.kPath;
		double Pc1x = startpoint.getX();
		double hEndpointControl1 = Math.sqrt(Math.pow((endpoint.getY()-Pc1y), 2)+(Math.pow(endpoint.getX()-Pc1x, 2)));
		double thetaEndpointControl1 = FastMath.asin((endpoint.getY()-Pc1y)/hEndpointControl1);
		double Pc2x = hEndpointControl1 * FastMath.cos(thetaEndpointControl1);
		double phi = RobotMap.gyroAngleWrapper.pidGet();
		double thetaRobotToSurface = getThetaSurface() - phi;

		Pair[] controlPoints;
		double Pc2y = Pc1y + FastMath.tan(thetaRobotToSurface)*(Pc2x-Pc1x);
		if ((phi > thetaAlignmentLine && Pc1x < Pc2x) || (phi < thetaAlignmentLine && Pc2x < Pc1x)) {
			controlPoints = new Pair[2];
			controlPoints[0] = new Pair(Pc1x, Pc1y);
			controlPoints[1] = new Pair(Pc2x, Pc2y);
		} else {
			double thetaAcrossHypotenuse = Math.PI - thetaRobotToSurface;
			double hypot = Math.sqrt(Math.pow((endpoint.getY()-startpoint.getY()), 2) + Math.pow((endpoint.getX()-startpoint.getX()), 2));
			double thetaInvented = FastMath.acos(endpoint.getY()/hypot);
			double thetaComplement = Math.PI/2 - thetaInvented;
			double endpointConnectedSide =  FastMath.sin(thetaComplement)*hypot/FastMath.sin(thetaAcrossHypotenuse);
			double thetaEndpoint = Math.PI - thetaComplement - thetaAcrossHypotenuse;
			double thetaRobotToSurfaceComplement = Math.PI/2 - thetaRobotToSurface;
			double cx = endpoint.getX() - (FastMath.cos(thetaRobotToSurfaceComplement) * endpointConnectedSide);
			double yEndpointTriangle = FastMath.sin(thetaRobotToSurfaceComplement) * endpointConnectedSide;
			double cy = endpoint.getY() - yEndpointTriangle;
			controlPoints = new Pair[1];
			controlPoints[0] = new Pair(cx, cy);

		}
		return controlPoints;
	}

	public static Pair[] meterRule(){
		if(RobotMap.lidar.getDistance() < 40.7){
			//back up till lidar.getDistance >= 1 meter
			double dist = 40.7 - RobotMap.lidar.getDistance();
			return (new Pair[]{new Pair(0, 0), new Pair(0, -dist)});
		}

		return null;

	}
}

