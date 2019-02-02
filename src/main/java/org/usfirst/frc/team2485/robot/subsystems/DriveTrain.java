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
	private static final double THETA_CAMERA = Math.PI/3;
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
	public TransferNode angleSetpointTN;
	public TransferNode angVelSetpointTN;
	public TransferNode velocitySetpointTN;
	public TransferNode velocityTN;
	public TransferNode angVelTN;
	public TransferNode distanceSetpointTN;


	//public PIDSourceWrapper kp_distancePIDSource;
	public PIDSourceWrapper distancePIDSource;
	public PIDSourceWrapper velocityPIDSource;
	public PIDSourceWrapper anglePIDSource;
	public PIDSourceWrapper angVelPIDSource;
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
		angVelSetpointTN = new TransferNode(0);
		angleSetpointTN = new TransferNode(0);
		distanceSetpointTN = new TransferNode(0);

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
		distancePID.setSetpointSource(distanceSetpointTN);

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
		anglePID.setSetpointSource(angleSetpointTN);
		anglePID.setContinuous(true);
		anglePID.setInputRange(-MAX_ANG, MAX_ANG);
		anglePID.setOutputRange(-MAX_ANGVEL, MAX_ANGVEL);
		anglePID.setOutputs(angVelSetpointTN);

		angVelPIDSource.setPidSource(()-> {
			return RobotMap.gyroRateWrapper.pidGet();
		});

		angVelPID.setSources(angVelPIDSource);
		angVelPID.setSetpointSource(angVelSetpointTN);
		angVelPID.setOutputRange(-MAX_CURR, MAX_CURR);
		angVelPID.setOutputs(angVelTN);

		leftCurrentPIDSource.setPidSource(() -> {
			return velocityTN.pidGet() + angVelTN.pidGet(); 
		});

		rightCurrentPIDSource.setPidSource(() -> {
			return velocityTN.pidGet() - angVelTN.pidGet();
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
    
   
	
	public static double getThetaAlignmentLine() {
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

		return theta + Math.PI/2;
	}

	public static Pair getAutoAlignEndpoint(double lidar, double target1, double target2) {
		double lidarDist = lidar;
		double thetaSurface = getThetaAlignmentLine(); //this is fine just leave it alone -S
		double phi = RobotMap.gyroAngleWrapper.pidGet();
		double cx1 = target1; 
		double cx2 = target2; 
		double thetaCamera = THETA_CAMERA; //angle of end of field of view to plane parallel to robot
		double thetaFieldOfView = Math.PI - thetaCamera*2; //total field of view angle
		double fovWidth = FastMath.tan(thetaFieldOfView/2)*lidarDist*2; //width in inches of total field of view (assuming lidar dist as height)
		double thetaRobotToSurface = thetaSurface - phi; //angle between plane of surface and plane parallel to front of robot
		System.out.println("Theta R2S: " + thetaRobotToSurface);
		double cx = (cx1+cx2)/2; //distance to alignment line in pixels (x-axis)
		double dx = cx*fovWidth/CAMERA_PIXEL_WIDTH; //distance to alignment line in inches 
		System.out.println("fovWidth/CAMERA_PIXEL_WIDTH: " + fovWidth/CAMERA_PIXEL_WIDTH);
		double phiInDegrees = Math.toDegrees(phi) % 360;
		dx -= fovWidth/2;
		System.out.println("dx: " + dx);
		System.out.println("Sin phi + 90: " + FastMath.sin(phi+Math.PI/2));
		double Px = ((phiInDegrees + 90 > 2 || phiInDegrees + 90 < -2) ? dx/FastMath.sin(phi + Math.PI/2) : 0) + ((phiInDegrees > 2 || phiInDegrees < -2) ? (lidarDist/FastMath.sin(phi)) : 0);
		System.out.println("Sin phi + Math.PI/2: " + FastMath.sin(phi+Math.PI/2));
		System.out.println("Sin phi: " + FastMath.sin(phi));
		double cy = (((phiInDegrees < 88 || phiInDegrees > 92) && (phiInDegrees > 272 || phiInDegrees < 268)) ? FastMath.tan(thetaRobotToSurface) : 0) * Px; 
		double Py = cy + (((phiInDegrees < 88 || phiInDegrees > 92) && (phiInDegrees > 272 || phiInDegrees < 268)) ? lidarDist/FastMath.cos(phi) : 0);


		return new Pair(Px, Math.abs(Py));
 
	}

	public boolean driveTo(double distance, double maxSpeed, double angle, double curvature, double toleranceDist, double toleranceAngle) {
		anglePID.enable();
		distancePID.enable();
		velocityPID.enable();
		angVelPID.enable();
		velocityRampRate.enable();
		leftMotorSetter.enable();
		rightMotorSetter.enable();
		angleSetpointTN.setOutput(angle);
		distanceSetpointTN.setOutput(distance);
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

	public static Pair[] generateControlPoints(double lidar, double target1, double target2) {
		Pair endpoint = getAutoAlignEndpoint(lidar, target1, target2);
		Pair startpoint = new Pair(0,0);
		double phi = RobotMap.gyroAngleWrapper.pidGet();
		double phiInDegrees = Math.toDegrees(phi) % 360;
		double thetaAlignmentLine = (Math.PI/2) - getThetaAlignmentLine();
		double Pc1y = (((endpoint.getY() - startpoint.getY())*ConstantsIO.kPath) + startpoint.getY());
		System.out.println("Startpoint Y: " + startpoint.getY());
		System.out.println("End Y - Start Y: " + (endpoint.getY() - startpoint.getY()));
		System.out.println("kPath * End-Start: " + (endpoint.getY() - startpoint.getY()) * ConstantsIO.kPath);
		double Pc1x = startpoint.getX() + (((phiInDegrees > 2 || phiInDegrees < -2) && (phiInDegrees > 182 || phiInDegrees < 178)) ? (Pc1y * FastMath.sin(phi))/FastMath.cos(phi) : 0);


		double hypot = Math.sqrt(Pc1y * Pc1y + Pc1x * Pc1x);
		double c2Hypot = lidar - hypot;

		System.out.println("phi in degrees: " + phiInDegrees);
	
		double Pc2x = ((phiInDegrees > 2 || phiInDegrees < -2) && (phiInDegrees > 182 || phiInDegrees < 178)) ? c2Hypot/FastMath.sin(phi) : 0;
		double Pc2y = ((phiInDegrees < 88 || phiInDegrees > 92) && (phiInDegrees > 272 || phiInDegrees < 268)) ? c2Hypot/FastMath.cos(phi) : 0;
		System.out.println("Pc2x: " + Pc2x);
		System.out.println("Pc2y: " + Pc2y);

		double thetaRobotToSurface = getThetaAlignmentLine() - phi;


		double Pc2r2sx = Pc2x*FastMath.cos(thetaRobotToSurface);
		double Pc2r2sy = Pc2y*FastMath.sin(thetaRobotToSurface);
		System.out.println("Pc2r2sx: " + Pc2r2sx);
		System.out.println("Pc2r2sy: " + Pc2r2sy);


		double thetaSurfaceDegrees = Math.toDegrees(getThetaAlignmentLine()) % 360;

		Pc2r2sx = ((thetaSurfaceDegrees > 2 || thetaSurfaceDegrees<0) && (thetaSurfaceDegrees > 182 || thetaSurfaceDegrees < 178)) ? Pc2r2sx / FastMath.sin(getThetaAlignmentLine()) : 0;
		Pc2r2sy = ((thetaSurfaceDegrees < 88 || thetaSurfaceDegrees > 92) && (thetaSurfaceDegrees > 272 || thetaSurfaceDegrees < 268)) ? Pc2r2sy / FastMath.cos(getThetaAlignmentLine()) : 0;

		Pc2x += Pc2r2sx;
		Pc2y += Pc2r2sy;

		Pc2y = endpoint.getY() - Pc2y;

		Pc2x = endpoint.getX() - Pc2x;

		// double hEndpointControl1 = Math.sqrt(Math.pow((endpoint.getY()-Pc1y), 2)+(Math.pow(endpoint.getX()-Pc1x, 2)));
		// double thetaEndpointControl1 = FastMath.asin((endpoint.getY()-Pc1y)/hEndpointControl1);
		// double Pc2x = hEndpointControl1 * FastMath.cos(thetaEndpointControl1);

		Pair[] controlPoints;
		// double Pc2y = Pc1y + FastMath.tan(thetaRobotToSurface)*(Pc2x-Pc1x);
		if ((phi > thetaAlignmentLine && Pc1x < Pc2x) || (phi < thetaAlignmentLine && Pc2x < Pc1x)) {
			controlPoints = new Pair[2];
			controlPoints[0] = new Pair(Pc1x, Pc1y);
			controlPoints[1] = new Pair(Pc2x, Pc2y);
		}
		// } else {
		// 	double thetaAcrossHypotenuse = Math.PI - thetaRobotToSurface;
		// 	double hypot = Math.sqrt(Math.pow((endpoint.getY()-startpoint.getY()), 2) + Math.pow((endpoint.getX()-startpoint.getX()), 2));
		// 	double thetaInvented = FastMath.acos(endpoint.getY()/hypot);
		// 	double thetaComplement = Math.PI/2 - thetaInvented;
		// 	double endpointConnectedSide =  FastMath.sin(thetaComplement)*hypot/FastMath.sin(thetaAcrossHypotenuse);
		// 	double thetaEndpoint = Math.PI - thetaComplement - thetaAcrossHypotenuse;
		// 	double thetaRobotToSurfaceComplement = Math.PI/2 - thetaRobotToSurface;
		// 	double cx = endpoint.getX() - (FastMath.cos(thetaRobotToSurfaceComplement) * endpointConnectedSide);
		// 	double yEndpointTriangle = FastMath.sin(thetaRobotToSurfaceComplement) * endpointConnectedSide;
		// 	double cy = endpoint.getY() - yEndpointTriangle;
		// 	// controlPoints = new Pair[1];
		// 	// controlPoints[0] = new Pair(cx, cy);

		// }
		controlPoints = new Pair[2];
		controlPoints[0] = new Pair(Pc1x, Math.abs(Pc1y));
		controlPoints[1] = new Pair(Pc2x, Math.abs(Pc2y));
		return controlPoints;
	}

	public static Pair[] meterRule(double lidar){
		if(RobotMap.lidar.getDistance() < 40.7){
			//back up till lidar.getDistance >= 1 meter
			double dist = 40.7 - lidar;
			return (new Pair[]{new Pair(0, 0), new Pair(0, -dist)});
		}

		return null;

	}
}

