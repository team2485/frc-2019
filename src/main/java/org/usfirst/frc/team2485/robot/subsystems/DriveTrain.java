package org.usfirst.frc.team2485.robot.subsystems;

import java.util.ArrayList;

import org.usfirst.frc.team2485.robot.Robot;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.FastMath;
import org.usfirst.frc.team2485.util.MotorSetter;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;
import org.usfirst.frc.team2485.util.AutoPath.Pair;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveTrain extends Subsystem {
    private static final double THETA_CAMERA = Math.PI/3;
	private static final double CAMERA_PIXEL_WIDTH = 320;
	private static final double cameraFieldOfView = Math.toRadians(65);

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

	public PIDSourceWrapper kPDistancePIDSource;
    public PIDSourceWrapper distancePIDSource;
    public PIDSourceWrapper velocityPIDSource;
    public PIDSourceWrapper leftCurrentPIDSource;
    public PIDSourceWrapper rightCurrentPIDSource;

    public MotorSetter leftMotorSetter;
    public MotorSetter rightMotorSetter;

    public double distBetweenCenterTargets;

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

		kPDistancePIDSource = new PIDSourceWrapper();
        distancePIDSource = new PIDSourceWrapper();
        velocityPIDSource = new PIDSourceWrapper();
        leftCurrentPIDSource = new PIDSourceWrapper();
        rightCurrentPIDSource = new PIDSourceWrapper();

        leftMotorSetter = new MotorSetter();
        rightMotorSetter = new MotorSetter();

		// kPDistancePIDSource.setPidSource(() -> {
		// 	return Math.min(ConstantsIO.kPMax_Distance,
		// 			FastMath.sqrt(2 * ConstantsIO.accelerationMax / Math.abs(distancePID.getError())));
		// });

        distancePIDSource.setPidSource(() -> {
            return (-RobotMap.driveLeftEncoderWrapperDistance.pidGet() + RobotMap.driveRightEncoderWrapperDistance.pidGet())/2;
        });

        distancePID.setSetpointSource(distanceSetpointTN);
        distancePID.setSources(distancePIDSource);
        distancePID.setOutputRange(-MAX_VELOCITY, MAX_VELOCITY);
		distancePID.setOutputs(distanceOutputTN);
		// distancePID.setConstantsSources(kPDistancePIDSource, null, null, null);

        velocityPIDSource.setPidSource(() -> {
            return (-RobotMap.driveLeftEncoderWrapperRate.pidGet() + RobotMap.driveRightEncoderWrapperRate.pidGet())/2;
        });

        velocityPID.setSetpointSource(distanceOutputTN);
        velocityPID.setSources(velocityPIDSource);
        velocityPID.setOutputRange(-ConstantsIO.driveTrainIMax, ConstantsIO.driveTrainIMax);
        velocityPID.setOutputs(velocityOutputTN);

        anglePID.setSetpointSource(angleSetpointTN);
        anglePID.setSources(RobotMap.gyroAngleWrapper);
        anglePID.setOutputRange(-ConstantsIO.driveTrainIMax, ConstantsIO.driveTrainIMax);
		anglePID.setOutputs(angVelOutputTN);
		anglePID.setContinuous(true);
		anglePID.setInputRange(0, 2 * Math.PI);
		
		angVelPID.setSetpointSource(null);
		angVelPID.setSources(RobotMap.gyroRateWrapper);
		angVelPID.setOutputRange(-ConstantsIO.driveTrainIMax, ConstantsIO.driveTrainIMax);
		angVelPID.setOutputs(null);


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

        distBetweenCenterTargets = distInchesCenterVisionTarget();


    }

    public void WarlordsDrive(double throttle, double steering, boolean quickTurn) {
        if(quickTurn) {
            RobotMap.driveLeftPercentOutput.set(steering);
            RobotMap.driveRightPercentOutput.set(-steering);
        } else {
            double left = throttle + Math.abs(throttle)*steering;
            double right = throttle - Math.abs(throttle)*steering;

            if(Math.abs(left) > ConstantsIO.driveTrainIMax) {
                right /= Math.abs(left);
                left /= Math.abs(left);
            } else if (Math.abs(right) > ConstantsIO.driveTrainIMax) {
                left /= Math.abs(right);
                right /= Math.abs(right);
            }

            RobotMap.driveLeftCurrent.set(left);
            RobotMap.driveRightCurrent.set(right);
        }
    }

    public void initDefaultCommand() {
       setDefaultCommand(new DriveWithControllers());
    }

    public void updateConstants() {
		distancePID.setPID(ConstantsIO.kPMax_Distance, 0, 0);
		velocityPID.setPID(ConstantsIO.kP_DriveVelocity, ConstantsIO.kI_DriveVelocity, ConstantsIO.kD_DriveVelocity, ConstantsIO.kF_DriveVelocity);
        anglePID.setPID(ConstantsIO.kP_DriveAngle, ConstantsIO.kI_DriveAngle, ConstantsIO.kD_DriveAngle);
		angVelPID.setPID(ConstantsIO.kP_DriveAngVel, ConstantsIO.kI_DriveAngVel, ConstantsIO.kD_DriveAngVel, ConstantsIO.kF_DriveAngVel);
		angVelPID.setFrictionTerm(ConstantsIO.kV_DriveAngVel, 0.5);
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
		// return Math.PI/2;
    }
    



    public boolean driveTo(double distance, double maxSpeed, double angle, double curvature, double toleranceDist, double toleranceAngle) {
		anglePID.enable();
		distancePID.enable();
		velocityPID.enable();
		// angVelPID.enable();
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
		// angVelPID.enable();
		//velocityRampRate.enable();
		


		velocityPID.setSetpointSource(null);
		velocityPID.setSetpoint(linearVelocity);
		angVelPID.setSetpointSource(null);
		angVelPID.setSetpoint(angularVelocity);

	}

	public boolean setAngle(double angle, double tolerance) {
		velocityPID.disable();
		anglePID.enable();
		distancePID.disable();
		//velocityRampRate.disable();
		// angVelPID.enable();
		leftMotorSetter.enable();
		rightMotorSetter.enable();
		anglePID.setAbsoluteTolerance(tolerance);
		angleSetpointTN.setOutput(angle);
		return anglePID.isOnTarget();

    }
    
    public static Pair getAutoAlignEndpoint(double lidar, double target1, double target2) {
		double thetaSurface = getThetaAlignmentLine(); //this is fine just leave it alone -S
		double phi = RobotMap.gyroAngleWrapper.pidGet();
		// double thetaCamera = THETA_CAMERA; //angle of end of field of view to plane parallel to robot
		// double thetaFieldOfView = Math.PI - thetaCamera*2; //total field of view angle
		// double fovWidth = FastMath.tan(thetaFieldOfView/2)*lidarDist*2; //width in inches of total field of view (assuming lidar dist as height)
		double thetaRobotToSurface = thetaSurface - phi; //angle between plane of surface and plane parallel to front of robot

		// // System.out.println("Theta R2S: " + thetaRobotToSurface);
		// double cx = (cx1+cx2)/2; //distance to alignment line in pixels (x-axis)
		// double dx = cx*fovWidth/CAMERA_PIXEL_WIDTH; //distance to alignment line in inches 
		// // System.out.println("fovWidth/CAMERA_PIXEL_WIDTH: " + fovWidth/CAMERA_PIXEL_WIDTH);
		// double phiInDegrees = Math.toDegrees(phi) % 360;
		// dx -= fovWidth/2;

		// //METHOD 1: 
		// // System.out.println("dx: " + dx);
		// // System.out.println("Sin phi + 90: " + FastMath.sin(phi+Math.PI/2));
		// double Px = ((phiInDegrees + 90 > 2 || phiInDegrees + 90 < -2) ? dx/FastMath.sin(phi + Math.PI/2) : 0) + ((phiInDegrees > 2 || phiInDegrees < -2) ? (lidarDist/FastMath.sin(phi)) : 0);
		// // System.out.println("Sin phi + Math.PI/2: " + FastMath.sin(phi+Math.PI/2));
		// // System.out.println("Sin phi: " + FastMath.sin(phi));
		// System.out.println("FastMath.cos(phi): " + FastMath.cos(phi));
		// System.out.println("FastMath.tan(thetaR2S): " + FastMath.tan(thetaRobotToSurface));
		// double cy = (((phiInDegrees < 88 || phiInDegrees > 92) && (phiInDegrees > 272 || phiInDegrees < 268)) ? FastMath.tan(thetaRobotToSurface) : 0) * Px; 
		// double Py = cy + (((phiInDegrees < 88 || phiInDegrees > 92) && (phiInDegrees > 272 || phiInDegrees < 268)) ? lidarDist/FastMath.cos(phi) : 0);

		// System.out.println("cy: " + cy);
		// System.out.println("Py: " + Py);


		// //METHOD 2: 2/3/19 -il, sk
		// double piMinusPhiDegrees = Math.toDegrees(Math.PI-phi) % 360;
		// double weirdSurfaceThingDegrees = Math.toDegrees((Math.PI/2) - (getThetaAlignmentLine()-Math.PI/2)) % 360;

		// double d = dx / FastMath.cos(thetaRobotToSurface);
		// System.out.println(d);
		// double Ly = lidar * ((piMinusPhiDegrees > 92 || piMinusPhiDegrees < 88) ? FastMath.cos(Math.PI - phi) : 0);
		// double Sy = d * ((weirdSurfaceThingDegrees > 92 || weirdSurfaceThingDegrees < 88) ? FastMath.cos((Math.PI/2) - (getThetaAlignmentLine()-Math.PI/2)) : 0);
		// double Py = Ly + Sy;

		// System.out.println("Weird Surface Thing Degrees: " + weirdSurfaceThingDegrees);
		// System.out.println("Boolean 1: " + (piMinusPhiDegrees > 2 || piMinusPhiDegrees < -2));
		// System.out.println("Boolean 2: " + (weirdSurfaceThingDegrees > 2 || weirdSurfaceThingDegrees < -2));

		// double Lx = lidar * ((piMinusPhiDegrees > 2 || piMinusPhiDegrees < -2) ? FastMath.sin(Math.PI - phi) : 0);
		// double Sx = d * ((weirdSurfaceThingDegrees > 2 || weirdSurfaceThingDegrees < -2) ? FastMath.sin((Math.PI/2) - (getThetaAlignmentLine()-Math.PI/2)) : 0);
		// double Px = Lx + Sx;


		//Method 3: 2/12/19 -il, sk, ag
		double focalLength = Robot.IMG_WIDTH / (2 * FastMath.tan(cameraFieldOfView/2));
		double centerX = Robot.IMG_WIDTH/2 - 0.5;
		double thetaCamera1 = FastMath.atan((target1 - centerX)/focalLength);
		double thetaCamera2 = FastMath.atan((target2-centerX)/focalLength);
		double thetaCamera = (thetaCamera1 + thetaCamera2) / 2;

		double dxTemp = lidar * FastMath.tan(thetaCamera);
		double thetaWTF = Math.PI - ((Math.PI/2) - thetaCamera) - thetaRobotToSurface;
		double dh = dxTemp * FastMath.sin((Math.PI/2) - thetaCamera)/FastMath.sin(thetaWTF);
		double dx = dh * FastMath.cos(thetaRobotToSurface);
		double thetaCameraDegrees = Math.toDegrees(thetaCamera) ;
		double hypot = thetaCameraDegrees > 2 || thetaCameraDegrees < -2 ?  dx / FastMath.sin(thetaCamera) : lidar - dh * FastMath.sin(thetaRobotToSurface);

		double Px = hypot * FastMath.sin(phi - thetaCamera);
		double Py = hypot * FastMath.cos(phi - thetaCamera);


		return new Pair(Px, Math.abs(Py));
 
	}

	public static Pair[] generateControlPoints(double lidar, double target1, double target2) {
		Pair endpoint = getAutoAlignEndpoint(lidar, target1, target2);
		// System.out.println("Lidar: " + lidar);
		// System.out.println("Endpoint: " + endpoint.getX() + ", " + endpoint.getY());
		Pair startpoint = new Pair(0,0);
		double phi = RobotMap.gyroAngleWrapper.pidGet();
		double phiInDegrees = Math.toDegrees(phi) % 360;
		double thetaAlignmentLine = (Math.PI/2) - getThetaAlignmentLine();
		double Pc1y = (((endpoint.getY() - startpoint.getY())*ConstantsIO.kPath) + startpoint.getY());
		// System.out.println("Startpoint Y: " + startpoint.getY());
		// System.out.println("End Y - Start Y: " + (endpoint.getY() - startpoint.getY()));
		// System.out.println("kPath * End-Start: " + (endpoint.getY() - startpoint.getY()) * ConstantsIO.kPath);
		double Pc1x = startpoint.getX() + (((phiInDegrees > 2 || phiInDegrees < -2) && (phiInDegrees > 182 || phiInDegrees < 178)) ? (Pc1y * FastMath.sin(phi))/FastMath.cos(phi) : 0);


		double hypot = Math.sqrt(Pc1y * Pc1y + Pc1x * Pc1x);
		double c2Hypot = lidar - hypot;

		// System.out.println("phi in degrees: " + phiInDegrees);
	
		double Pc2x = ((phiInDegrees > 2 || phiInDegrees < -2) && (phiInDegrees > 182 || phiInDegrees < 178)) ? c2Hypot/FastMath.sin(phi) : 0;
		double Pc2y = ((phiInDegrees < 88 || phiInDegrees > 92) && (phiInDegrees > 272 || phiInDegrees < 268)) ? c2Hypot/FastMath.cos(phi) : 0;
		// System.out.println("Pc2x: " + Pc2x);
		// System.out.println("Pc2y: " + Pc2y);

		double thetaRobotToSurface = getThetaAlignmentLine() - phi;


		double Pc2r2sx = Pc2x*FastMath.cos(thetaRobotToSurface);
		double Pc2r2sy = Pc2y*FastMath.sin(thetaRobotToSurface);
		// System.out.println("Pc2r2sx: " + Pc2r2sx);
		// System.out.println("Pc2r2sy: " + Pc2r2sy);


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
		



		//  System.out.println("CP 1: " + controlPoints[0].getX() + ", " + controlPoints[0].getY());
		//  System.out.println("CP 2: " + controlPoints[1].getX() + ", " + controlPoints[1].getY());

		return controlPoints;

		
    }
    
    public static Pair[] generateSandstormControlPoints(boolean cargoSidePath) {
        Pair[] controlPoints;
        // RobotMap.driveLeftEncoder.reset();
        // RobotMap.driveRightEncoder.reset();
        double endpointX;
        double endpointY;



        Pair startPosition; //keep for visualization
        //everything relative to (0,0) @ start of path/auto

        

        if(cargoSidePath) {
            startPosition = new Pair(0,0);
            endpointX = -22.5;
            endpointY = 155.5;
            controlPoints = new Pair[1];
            controlPoints[0] = new Pair(0, 155.5); 
            return controlPoints;
        } 
        else { //front
            double startPosX = 83.06; //change startPos x, y 
            double startPosY = -80;
            startPosition = new Pair(startPosX, startPosY); 
            endpointX = -71.12;
            endpointY = 109.65;
            controlPoints = new Pair[2];
            controlPoints[0] = new Pair(startPosX, (endpointY-startPosY) * ConstantsIO.kPath);
            controlPoints[1] = new Pair(endpointX, (endpointY-startPosY) * ConstantsIO.kPath);
            return controlPoints;
        }
        
    }

    public static Pair getSandstormEndpoint(boolean cargoSide){
        Pair endPosition;
        if(cargoSide){
            endPosition = new Pair(-22.5, 155.5);
            return endPosition;
        } else {
            endPosition = new Pair(-71.12, 109.65);
            return endPosition;
        }

    }

	public double distInchesCenterVisionTarget() {
		double theta = Math.toRadians(14.5);
		double height = FastMath.cos(theta) * 5.5;
		height /= 2;
		double xh = FastMath.tan(theta) * height;
		xh += xh + 8;
		return xh;
	}

	public double[] collectSamples() {
		double thetaRobotToSurface = getThetaAlignmentLine() - RobotMap.gyroAngleWrapper.pidGet();
		double dpx = distBetweenCenterTargets * FastMath.cos(thetaRobotToSurface);
		dpx += 2;
		ArrayList<Double> arr = Robot.samples;
		double val1 = 0, val2 = 0;
		val1 = arr.get(0);
		if (Robot.samples.size() >= 10) {
			for(int i = 1; i < 10; i++) {
				double currVal = arr.get(i);
				if(currVal - dpx > val1 && currVal + dpx < val1) {
					val1 = (val1 + currVal)/2;
				} else if (val2 == 0) {
					val2 = currVal;
				} else {
					val2 = (val2 + currVal) / 2;
				}
			}
		}
		double[] samples = {val1, val2};
		return samples;

	}


	// public boolean visionTargetsAreVisible() {

	// 	if (!Robot.contoursVisible) {
	// 		return false;
	// 	}

	// 	return samples;

	// 	double[] samples = collectSamples();
	// 	double thetaRobotToSurface = getThetaAlignmentLine() - RobotMap.gyroAngleWrapper.pidGet();
	// 	double dpx = distBetweenCenterTargets * FastMath.cos(thetaRobotToSurface);

	// 	if (Math.abs(samples[1] - samples[0]) > dpx + 2 && Math.abs(samples[1] - samples[0]) < dpx - 2) {
	// 		return false;
	// 	} 
		
	// 	return true;
	// }

	public static Pair[] meterRule(double lidar){
		if(RobotMap.lidar.getDistance() < 40.7){
			//back up till lidar.getDistance >= 1 meter
			double dist = 40.7 - lidar;
			return (new Pair[]{new Pair(0, 0), new Pair(0, -dist)});
		}

		return null;

	}

    public void enablePID(boolean enabled) {
        if (enabled) {
            distancePID.enable();
            velocityPID.enable();
            anglePID.enable();
            // angVelPID.enable();
            leftMotorSetter.enable();
            rightMotorSetter.enable();
        } else {
            distancePID.disable();
            velocityPID.disable();
            anglePID.disable();
            // angVelPID.disable();
            leftMotorSetter.disable();
            rightMotorSetter.disable();
        }
    }
}