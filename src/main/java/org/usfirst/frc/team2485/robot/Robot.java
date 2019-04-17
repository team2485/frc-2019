package org.usfirst.frc.team2485.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
//import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commandGroups.SandstormAuto;
import org.usfirst.frc.team2485.robot.commands.CargoArmWithControllers;
import org.usfirst.frc.team2485.robot.commands.DriveStraight;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;
import org.usfirst.frc.team2485.robot.subsystems.CargoArm;
import org.usfirst.frc.team2485.robot.subsystems.CargoRollers;
import org.usfirst.frc.team2485.robot.subsystems.DriveTrain;
import org.usfirst.frc.team2485.robot.subsystems.Elevator;
import org.usfirst.frc.team2485.robot.subsystems.HatchIntake;
import org.usfirst.frc.team2485.util.AutoPath;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.EncoderWrapperRateAndDistance;
import org.usfirst.frc.team2485.util.FastMath;
import org.usfirst.frc.team2485.util.GripPipeline;
import org.usfirst.frc.team2485.util.MotorSetter;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.PigeonWrapperRateAndAngle;
import org.usfirst.frc.team2485.util.RampRate;
import org.usfirst.frc.team2485.util.SurajPipeline;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;
import org.usfirst.frc.team2485.util.WarlordsPIDControllerSystem;
import edu.wpi.first.networktables.NetworkTable;

public class Robot
extends TimedRobot {
    public static final int IMG_WIDTH = 320;
    public static final int IMG_HEIGHT = 240;
    private VisionThread visionThread;
    public static double centerX = 7;
    public static ArrayList<Double> samples;
    public static boolean doneCollecting;
    public static boolean restart;
    public static AutoPath.Pair[] controlPoints;
    public AutoPath.Pair endpoint;
    private static AutoPath path;
    private final Object imgLock = new Object();
    Command m_autonomousCommand;
    SendableChooser<Command> m_chooser = new SendableChooser();
    public static boolean collectingSamples;
    public static boolean contoursVisible;
    boolean ejecting = false;
    public static Command auto;
    public CvSource output;
  
    private SerialPort jevois = null;
    private int loopCount;
    private UsbCamera jevoisCam;
    private MjpegServer jevoisServer;
    public static NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  

    @Override
    public void robotInit() {
        FastMath.init();
        ConstantsIO.init();
        RobotMap.init();
        OI.init();
        SandstormAuto.init(false);
        auto = new SandstormAuto();
        restart = true;
       

        // int tryCount = 0;
        // do {
        //     try {
        //         System.out.print("Trying to create jevois SerialPort...");
        //         jevois = new SerialPort(9600, SerialPort.Port.kUSB);
        //         tryCount = 99;
        //         System.out.println("success!");
        //     } catch (Exception e) {
        //         tryCount += 1;
        //         System.out.println("failed!");
        //     }
        // } while (tryCount < 3);
        
        // if (tryCount == 99) {
        //     writeJeVois("info\n");
        // }
        // loopCount = 0;

        // System.out.println("Starting CameraServer");
        // if (jevoisCam == null) {
        //       jevoisCam = CameraServer.getInstance().startAutomaticCapture(1);
              
        //             jevoisCam.setVideoMode(PixelFormat.kYUYV,320,240,15);
        //             //jevoisCam.setPixelFormat(PixelFormat.kYUYV);
        //             VideoMode vm = jevoisCam.getVideoMode();
        //             System.out.println("jevoisCam pixel: " + vm.pixelFormat);
        //             System.out.println("jevoisCam res: " + vm.width + "x" + vm.height);
        //             System.out.println("jevoisCam fps: " + vm.fps);
        //         }
        //         // if (jevoisServer == null) {
        //         //     jevoisServer = new MjpegServer("JeVoisServer", 1181);
        //         //     jevoisServer.setSource(jevoisCam);
        //         // }


		// // VideoMode videoMode = new VideoMode(0, IMG_WIDTH, IMG_HEIGHT, 18);
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
        // // UsbCamera jevois = CameraServer.getInstance().startAutomaticCapture(1);

        camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
        camera.setFPS(18);
        // camera.setWhiteBalanceAuto();
        // UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        // UsbCamera jevois = CameraServer.getInstance().startAutomaticCapture();
        // camera.setVideoMode(videoMode);
        // camera.setPixelFormat(PixelFormat.kYUYV);

        // jevois.setPixelFormat(PixelFormat.kYUYV);
        // jevois.setExposureManual(20);
        
		// camera.setVideoMode(PixelFormat.kYUYV, 160, 120, 30);
	    // camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
	    		
		//  visionThread = new VisionThread(jevois, new GripPipeline(), pipeline -> {
        //     // System.out.println("boolean vision ting" + pipeline.filterContoursOutput().isEmpty());
        //     // System.out.println("contours size:" + pipeline.filterContoursOutput().size());
		//         if (!pipeline.filterContoursOutput().isEmpty()) {
        //             for(MatOfPoint centerXVal : pipeline.filterContoursOutput()){
		//             Rect r = Imgproc.boundingRect(centerXVal);
		//             synchronized (imgLock) {
        //                 centerX = r.x + (r.width / 2);
        //                 samples.add(centerX);
        //                 System.out.println("centerX" + centerX);
        //                 if(samples.size() > 10) {
        //                     samples.remove(0);
        //                 }
        //             }
        //         }
        //         }

                
               
        //         for(MatOfPoint centerXVal : pipeline.filterContoursOutput()){
        //             Rect r = Imgproc.boundingRect(centerXVal);
        //             centerX = r.x + (r.width / 2);
        //             System.out.println("centerX" + centerX);
        //             System.out.println("Num cont.:" + pipeline.filterContoursOutput().size() );
        //             System.out.println("centerx: " + centerX);
        //         }
              
        //             System.out.println("size:" + 0);
                
                
		//     });
        //     visionThread.start();
           


        output = CameraServer.getInstance().putVideo("Processed: ", 640, 480);

		
		VisionThread visionThread = new VisionThread(camera, new SurajPipeline(), pipeline -> {
			output.putFrame(pipeline.cvRectangleOutput());

		});
		visionThread.start();

    }

    // public void checkJeVois() {
    //     if (jevois == null) {
    //         System.out.println("No Jevois");
    //         return;
    //     }
    //     if (jevois.getBytesReceived() > 0) {
    //         System.out.println("Waited: " + loopCount + " loops, Rcv'd: " + jevois.readString());
    //         loopCount = 0;
    //     } 
    // }

    // public void writeJeVois(String cmd) {
    //     if (jevois == null) return;
    //     int bytes = jevois.writeString(cmd);
    //     System.out.println("wrote " +  bytes + "/" + cmd.length() + " bytes");    
    //     loopCount = 0;
    // }


    @Override
    public void disabledInit() {
        restart = true;

    }

    @Override
    public void disabledPeriodic() {
        
        //checkJeVois();
        Scheduler.getInstance().run();
        this.updateSmartDashboard();
        //RobotMap.elevatorEncoder.reset(); //LOOK

    }

    @Override
    public void autonomousInit() {
        ConstantsIO.init();
        RobotMap.driveTrain.updateConstants();
        RobotMap.hatchIntake.slideIn();
        RobotMap.hatchIntake.lift();
        RobotMap.cargoArmEncoder.reset();
        RobotMap.elevatorEncoder.reset();
        RobotMap.driveLeftEncoder.reset();
        RobotMap.driveRightEncoder.reset();
        RobotMap.gyroAngleWrapper.reset();
        RobotMap.gyroRateWrapper.reset();
        RobotMap.driveLeftTalon1.clearStickyFaults();
        RobotMap.driveLeftTalon2.clearStickyFaults();
        RobotMap.driveLeftTalon3.clearStickyFaults();
        RobotMap.driveLeftTalon4.clearStickyFaults();
        RobotMap.driveRightTalon1.clearStickyFaults();
        RobotMap.driveRightTalon2.clearStickyFaults();
        RobotMap.driveRightTalon3.clearStickyFaults();
        RobotMap.driveRightTalon4.clearStickyFaults();
        // RobotMap.driveTrain.enablePID(true);
        // RobotMap.cargoArm.enablePID(true);
        // SandstormAuto.init(true);


    }

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        this.updateSmartDashboard();
    }

    @Override
    public void teleopInit() {
        ConstantsIO.init();
        RobotMap.compressor.setClosedLoopControl(false);
        RobotMap.elevatorEncoder.reset();
        RobotMap.updateConstants();
        RobotMap.gyroAngleWrapper.reset();
        RobotMap.driveTrain.enablePID(false);
        // RobotMap.driveTrain.leftMotorSetter.setSetpointSource(RobotMap.driveTrain.angVelOutputTN);
        // RobotMap.driveTrain.rightMotorSetter.setSetpointSource(RobotMap.driveTrain.angV)
        // RobotMap.driveTrain.teleopLeftMotorSetter.enable();
        // RobotMap.driveTrain.teleopRightMotorSetter.enable();
        
        
       
        
        CargoArmWithControllers.init = true;

    }

    @Override
    public void teleopPeriodic() {
        if(restart){
        Scheduler.getInstance().run();
        this.updateSmartDashboard();
        if (RobotMap.cargoArmLimitSwitchUp.get()) {
            RobotMap.cargoArmEncoder.reset();
		}
        RobotMap.compressor.setClosedLoopControl(false);
        }
      
       
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");
    //read values periodically
        double x = tx.getDouble(99);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
//post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX: ", x);
        SmartDashboard.putBoolean("Dock", x != 99);




       // System.out.println(NetworkTableInstance.getDefault().getTable("limelight").getEntry("<variablename>").setNumber(<value>));
       
        //checkJeVois();
    }

    @Override
    public void testPeriodic() {
		RobotMap.compressor.setClosedLoopControl(true);
		
        RobotMap.driveTrain.enablePID(false);
		RobotMap.elevator.enablePID(false);
        this.updateSmartDashboard();
    }

    public void updateSmartDashboard() {
        // SmartDashboard.putString("ConstantsIO Last Modified: ", ConstantsIO.lastModified);
         SmartDashboard.putNumber("Elevator Distance PID Setpoint: ", RobotMap.elevator.distancePID.getSetpoint());
        // SmartDashboard.putNumber("Elevator Distance PID Error: ", RobotMap.elevator.distancePID.getError());
         SmartDashboard.putNumber("Elevator Setpoint TN: ", RobotMap.elevator.distanceSetpointTN.getOutput());
        SmartDashboard.putNumber("Elevator Setpoint Ramped TN: ", RobotMap.elevator.distanceSetpointRampedTN.getOutput());
        // SmartDashboard.putBoolean("Elevator Setpoint Ramp Rate Enabled: ", RobotMap.elevator.distanceSetpointRampRate.isEnabled());
         SmartDashboard.putBoolean("Elevator Distance PID Enabled: ", RobotMap.elevator.distancePID.isEnabled());
        SmartDashboard.putNumber("Elevator Position:", RobotMap.elevatorEncoderWrapperDistance.pidGet());
        // SmartDashboard.putNumber("Elevator Placement: ", RobotMap.elevatorEncoderWrapperDistance.pidGet() - 3.0);
        SmartDashboard.putNumber("Elevator Output Current: ", RobotMap.elevatorTalon1.getOutputCurrent());
         SmartDashboard.putNumber("Elevator Encoder Rate: ", RobotMap.elevatorEncoderWrapperRate.pidGet());
        SmartDashboard.putBoolean("arm Down Velocity PID Enabled: ", RobotMap.cargoArm.downVelocityPID.isEnabled());
        SmartDashboard.putNumber("arm Down Velocity Error: ", RobotMap.cargoArm.downVelocityPID.getError());
        SmartDashboard.putNumber("arm Down Velocity Setpoint: ", RobotMap.cargoArm.downVelocityPID.getSetpoint());
        SmartDashboard.putNumber("arm Encoder Rate", RobotMap.cargoArmEncoderWrapperRate.pidGet());
        SmartDashboard.putNumber("angVelOutputTN", RobotMap.driveTrain.angVelOutputTN.getOutput());
        // SmartDashboard.putNumber("ELevator Distance Output TN: ", RobotMap.elevator.distanceOutputTN.getOutput());
         SmartDashboard.putBoolean("Elevator Controller System Enabled: ", RobotMap.elevator.elevatorControllerSystem.isEnabled());
         SmartDashboard.putBoolean("Elevator Motor Setter: ", RobotMap.elevator.motorSetter.isEnabled());
        // SmartDashboard.putNumber("Cargo Arm Distance PID Error: ", RobotMap.cargoArm.distancePID.getError());
         SmartDashboard.putNumber("Cargo Arm Setpoint TN: ", RobotMap.cargoArm.distanceSetpointTN.getOutput());
        // SmartDashboard.putBoolean("Cargo Arm Distance PID Enabled: ", RobotMap.cargoArm.distancePID.isEnabled());
            SmartDashboard.putNumber("Cargo Arm Position: ", RobotMap.cargoArmEncoderWrapperDistance.pidGet());
        // SmartDashboard.putNumber("Cargo Arm PID Source Output: ", RobotMap.cargoArm.distanceOutputPIDSource.pidGet());
        // SmartDashboard.putBoolean("Cargo Arm on position:", RobotMap.cargoArm.distancePID.isOnTarget());
         SmartDashboard.putBoolean("Cargo Arm Up Limit Switch: ", RobotMap.cargoArmLimitSwitchUp.get());
        SmartDashboard.putBoolean("Cargo Arm Down Limit Switch: ", RobotMap.cargoArmLimitSwitchDown.get());
        // SmartDashboard.putNumber("Cargo Arm Output Current: ", RobotMap.cargoArmTalon.getOutputCurrent());
        // SmartDashboard.putNumber("Cargo Arm Distance PID Setpoint Please: ", RobotMap.cargoArm.distancePID.getSetpoint());
          SmartDashboard.putNumber("Cargo Arm Failsafe TN: ", RobotMap.cargoArm.failsafeTN.getOutput());
        // SmartDashboard.putNumber("Cargo Arm Error: ", RobotMap.cargoArm.distancePID.getError());
        // SmartDashboard.putNumber("Arm Ramped Setpoint:", RobotMap.cargoArm.distanceSetpointRampedTN.getOutput());
        // SmartDashboard.putNumber("up ramp arm:", ConstantsIO.armDistanceSetpointUpRamp);
             SmartDashboard.putNumber("Cargo Arm output current", RobotMap.cargoArmTalon.getOutputCurrent());
            // SmartDashboard.putNumber("drive steering", DriveWithControllers.
        // SmartDashboard.putNumber("Cargo Arm Distance Output TN ", RobotMap.cargoArm.distanceOutputTN.getOutput());
        // SmartDashboard.putNumber("Cargo Arm Ramped Distance Setpoint TN: ", RobotMap.cargoArm.distanceSetpointRampedTN.getOutput());
        // SmartDashboard.putNumber("Cargo Arm Ramped TN: ", RobotMap.cargoArm.distanceSetpointRampedTN.getOutput());
        // SmartDashboard.putNumber("Cargo Roller Current", RobotMap.cargoRollersTalon.getOutputCurrent());
        // SmartDashboard.putBoolean("Cargo Intaken: ", RobotMap.cargoRollers.intaken);
        // SmartDashboard.putNumber("Drive Train Distance Setpoint: ", RobotMap.driveTrain.distanceSetpointTN.pidGet());
        // SmartDashboard.putNumber("Drive Train Angle Setpoint: ", RobotMap.driveTrain.angleSetpointTN.pidGet());
        // SmartDashboard.putNumber("DT Control Point X: ", DriveTrain.generateSandstormControlPoints(true)[0].getX());
        // SmartDashboard.putNumber("DT Control Point Y: ", DriveTrain.generateSandstormControlPoints(true)[0].getY());
        // SmartDashboard.putNumber("DT Endpoint X: ", DriveTrain.getSandstormEndpoint(true).getX());
        // SmartDashboard.putNumber("DT Endpoint Y: ", DriveTrain.getSandstormEndpoint(true).getY());
        // SmartDashboard.putBoolean("Drive Train Distance PID Enabled: ", RobotMap.driveTrain.distancePID.isEnabled());
        // SmartDashboard.putBoolean("Drive Train Angle PID Enabled: ", RobotMap.driveTrain.anglePID.isEnabled());
        // SmartDashboard.putNumber("Drive Train Talon Current: ", RobotMap.driveLeftTalon1.getOutputCurrent());
        // SmartDashboard.putNumber("Drive Train Distance Output: ", RobotMap.driveTrain.distanceOutputTN.getOutput());
        // SmartDashboard.putNumber("Drive Train Velocity Output: ", RobotMap.driveTrain.velocityOutputTN.getOutput());
        // SmartDashboard.putNumber("Drive Train Velocity PID Setpoint: ", RobotMap.driveTrain.velocityPID.getSetpoint());
        // SmartDashboard.putNumber("Drive Train Velocity PID Source: ", RobotMap.driveTrain.velocityPIDSource.pidGet());
        // SmartDashboard.putNumber("Drive Train Distance Output TN: ", RobotMap.driveTrain.distanceOutputTN.getOutput());
        // SmartDashboard.putNumber("Drive Train Distance Error: ", RobotMap.driveTrain.distancePID.getError());
        // SmartDashboard.putNumber("Drive Train Velocity Error: ", RobotMap.driveTrain.velocityPID.getError());
        // SmartDashboard.putNumber("DriveTrain Distance Source: ", RobotMap.driveTrain.distancePIDSource.pidGet());
        SmartDashboard.putNumber("Drive Train Left Encoder Wrapper Dist: ", RobotMap.driveLeftEncoderWrapperDistance.pidGet());
        SmartDashboard.putNumber("Drive Train Right Encoder Wrapper Dist: ", RobotMap.driveRightEncoderWrapperDistance.pidGet());
        // SmartDashboard.putNumber("Drive Train Ang Vel Error: ", RobotMap.driveTrain.angVelPID.getError());
         SmartDashboard.putNumber("Drive Train Angle Error: ", RobotMap.driveTrain.anglePID.getError());
         SmartDashboard.putBoolean("left setter enabled",  RobotMap.driveTrain.teleopLeftMotorSetter.isEnabled());
         SmartDashboard.putBoolean("right setter enabled",  RobotMap.driveTrain.teleopRightMotorSetter.isEnabled());
         SmartDashboard.putNumber("Gyro Angle: ", RobotMap.gyroAngleWrapper.pidGet());
         SmartDashboard.putNumber("table.getEntry(tv).getDouble(0.0)", table.getEntry("tv").getDouble(0.0));

        // SmartDashboard.putNumber("Cargo Arm Talon Output", RobotMap.cargoArmTalon.getMotorOutputPercent());
        // SmartDashboard.putNumber("Drive Train Ang Vel Output", RobotMap.driveTrain.angVelOutputTN.getOutput());
        // SmartDashboard.putNumber("kP Ang Vel", ConstantsIO.kP_DriveAngVel);
        // SmartDashboard.putNumber("kP Ang Vel public version", RobotMap.driveTrain.angVelPID.kP);
        // SmartDashboard.putNumber("Gyro Value:", RobotMap.gyroAngleWrapper.pidGet());
        // SmartDashboard.putBoolean("Ang Vel Enabled: ", RobotMap.driveTrain.angVelPID.isEnabled());
        SmartDashboard.putNumber("Drive Talon Left 1 Current:", RobotMap.driveLeftTalon1.getOutputCurrent());
        SmartDashboard.putNumber("Drive Talon Left 2 Current:", RobotMap.driveLeftTalon2.getOutputCurrent());
        SmartDashboard.putNumber("Drive Talon Left 3 Current:", RobotMap.driveLeftTalon3.getOutputCurrent());
        SmartDashboard.putNumber("Drive Talon Left 4 Current:", RobotMap.driveLeftTalon4.getOutputCurrent());
        SmartDashboard.putNumber("Drive Talon Right 1 Current:", RobotMap.driveRightTalon1.getOutputCurrent());
        SmartDashboard.putNumber("Drive Talon Right 2 Current:", RobotMap.driveRightTalon2.getOutputCurrent());
        SmartDashboard.putNumber("Drive Talon Right 3 Current:", RobotMap.driveRightTalon3.getOutputCurrent());
        SmartDashboard.putNumber("Drive Talon Right 4 Current:", RobotMap.driveRightTalon4.getOutputCurrent());

        SmartDashboard.putBoolean("Limelight PID Enabled: ", RobotMap.driveTrain.limelightPID.isEnabled());
        SmartDashboard.putNumber("Limelight Output TN: ", RobotMap.driveTrain.limelightLeftRampedTN.getOutput());
        SmartDashboard.putNumber("Ang Vel TN: ", RobotMap.driveTrain.angVelOutputTN.getOutput());
        SmartDashboard.putNumber("Motor Setter Setter Source Left:", RobotMap.driveTrain.motorSetterSetterSourceLeft.pidGet());
        SmartDashboard.putNumber("Limelight PID Error: ", RobotMap.driveTrain.limelightPID.getError());
        
        SmartDashboard.putNumber("throttle", OI.getDriveThrottle());
        SmartDashboard.putNumber("steering", OI.getDriveSteering());
        SmartDashboard.putBoolean("angle is enabled?", RobotMap.driveTrain.anglePID.isEnabled());
        SmartDashboard.putNumber("dt angle+ setpt", RobotMap.driveTrain.anglePID.getSetpoint());
        SmartDashboard.putNumber("dt angle+ setpt limelight", RobotMap.driveTrain.limelightPID.getSetpoint());
        SmartDashboard.putNumber("dt angle error", RobotMap.driveTrain.anglePID.getError());
        //SmartDashboard.putNumber("dt angle setpt", RobotMap.driveTrain.anglePID.getSetpoint());


        // SmartDashboard.putBoolean("Drive Train Velocity Enabled: ", RobotMap.driveTrain.velocityPID.isEnabled());
        // SmartDashboard.putNumber("Distance Output PID Source: ", RobotMap.cargoArm.distanceOutputPIDSource.pidGet());
        // SmartDashboard.putBoolean("Lift Up: ", RobotMap.liftSolenoidOut.get());
        // SmartDashboard.putNumber("Suraj RYStick output: ", OI.getArmManual());
        // SmartDashboard.putNumber("Hatch Intake Rollers Current", RobotMap.hatchRollersTalon.getOutputCurrent());
        // SmartDashboard.putNumber("Cargo Arm Failsafe TN: ", RobotMap.cargoArm.failsafeTN.getOutput());
        
        SmartDashboard.putNumber("Drivetrain Left Velocity: ", RobotMap.driveLeftEncoderWrapperRate.pidGet());

        SmartDashboard.putNumber("Drivetrain Velocity Setpoint", RobotMap.driveTrain.teleopSetpointLeftTN.getOutput());

        SmartDashboard.putNumber("Scaled Error: ", RobotMap.driveTrain.teleopSetpointLeftRamp.scaledError);
      
    }
}