package org.usfirst.frc.team2485.robot;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
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
import org.usfirst.frc.team2485.util.MotorSetter;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.PigeonWrapperRateAndAngle;
import org.usfirst.frc.team2485.util.RampRate;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;
import org.usfirst.frc.team2485.util.WarlordsPIDControllerSystem;

public class Robot
extends TimedRobot {
    public static final int IMG_WIDTH = 64;
    public static final int IMG_HEIGHT = 48;
    private VisionThread visionThread;
    public static double centerX = 0.0;
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

    @Override
    public void robotInit() {
        FastMath.init();
        ConstantsIO.init();
        RobotMap.init();
        OI.init();
        SandstormAuto.init(false);
        auto = new SandstormAuto();
        restart = true;



		
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
        camera.setFPS(10);

    }

    @Override
    public void disabledInit() {
        restart = true;

    }

    @Override
    public void disabledPeriodic() {
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
        //RobotMap.driveTrain.distanceSetpointTN.setOutput(200);
        
       
        
        CargoArmWithControllers.init = true;

        // RobotMap.driveLeftTalonWrapperPercentOutput1.set(0.5);
        // RobotMap.driveLeftTalonWrapperPercentOutput2.set(0.5);
        // RobotMap.driveLeftTalonWrapperPercentOutput3.set(0.5);
        // RobotMap.driveLeftTalonWrapperPercentOutput4.set(0.5);

        // RobotMap.driveRightTalonWrapperPercentOutput1.set(0.5);
        // RobotMap.driveRightTalonWrapperPercentOutput2.set(0.5);
        // RobotMap.driveRightTalonWrapperPercentOutput3.set(0.5);
        // RobotMap.driveRightTalonWrapperPercentOutput4.set(0.5);




    }

    @Override
    public void teleopPeriodic() {
        // RobotMap.driveLeftTalonWrapperCurrent1.set(2);
        // RobotMap.driveLeftTalonWrapperCurrent2.set(2);

        if(restart){
        Scheduler.getInstance().run();
        this.updateSmartDashboard();
        if (RobotMap.cargoArmLimitSwitchUp.get()) {
            RobotMap.cargoArmEncoder.reset();
		}
        RobotMap.compressor.setClosedLoopControl(false);
        }
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
        //  SmartDashboard.putNumber("Elevator Distance PID Setpoint: ", RobotMap.elevator.distancePID.getSetpoint());
        // // SmartDashboard.putNumber("Elevator Distance PID Error: ", RobotMap.elevator.distancePID.getError());
        //  SmartDashboard.putNumber("Elevator Setpoint TN: ", RobotMap.elevator.distanceSetpointTN.getOutput());
        // SmartDashboard.putNumber("Elevator Setpoint Ramped TN: ", RobotMap.elevator.distanceSetpointRampedTN.getOutput());
        // // SmartDashboard.putBoolean("Elevator Setpoint Ramp Rate Enabled: ", RobotMap.elevator.distanceSetpointRampRate.isEnabled());
        //  SmartDashboard.putBoolean("Elevator Distance PID Enabled: ", RobotMap.elevator.distancePID.isEnabled());
        // SmartDashboard.putNumber("Elevator Position:", RobotMap.elevatorEncoderWrapperDistance.pidGet());
        // // SmartDashboard.putNumber("Elevator Placement: ", RobotMap.elevatorEncoderWrapperDistance.pidGet() - 3.0);
        // SmartDashboard.putNumber("Elevator Output Current: ", RobotMap.elevatorTalon1.getOutputCurrent());
        //  SmartDashboard.putNumber("Elevator Encoder Rate: ", RobotMap.elevatorEncoderWrapperRate.pidGet());
        // SmartDashboard.putBoolean("arm Down Velocity PID Enabled: ", RobotMap.cargoArm.downVelocityPID.isEnabled());
        // SmartDashboard.putNumber("arm Down Velocity Error: ", RobotMap.cargoArm.downVelocityPID.getError());
        // SmartDashboard.putNumber("arm Down Velocity Setpoint: ", RobotMap.cargoArm.downVelocityPID.getSetpoint());
        // SmartDashboard.putNumber("arm Encoder Rate", RobotMap.cargoArmEncoderWrapperRate.pidGet());
        // // SmartDashboard.putNumber("ELevator Distance Output TN: ", RobotMap.elevator.distanceOutputTN.getOutput());
        //  SmartDashboard.putBoolean("Elevator Controller System Enabled: ", RobotMap.elevator.elevatorControllerSystem.isEnabled());
        //  SmartDashboard.putBoolean("Elevator Motor Setter: ", RobotMap.elevator.motorSetter.isEnabled());
        // // SmartDashboard.putNumber("Cargo Arm Distance PID Error: ", RobotMap.cargoArm.distancePID.getError());
        //  SmartDashboard.putNumber("Cargo Arm Setpoint TN: ", RobotMap.cargoArm.distanceSetpointTN.getOutput());
        // // SmartDashboard.putBoolean("Cargo Arm Distance PID Enabled: ", RobotMap.cargoArm.distancePID.isEnabled());
        //     SmartDashboard.putNumber("Cargo Arm Position: ", RobotMap.cargoArmEncoderWrapperDistance.pidGet());
        // // SmartDashboard.putNumber("Cargo Arm PID Source Output: ", RobotMap.cargoArm.distanceOutputPIDSource.pidGet());
        // // SmartDashboard.putBoolean("Cargo Arm on position:", RobotMap.cargoArm.distancePID.isOnTarget());
        //  SmartDashboard.putBoolean("Cargo Arm Up Limit Switch: ", RobotMap.cargoArmLimitSwitchUp.get());
        // SmartDashboard.putBoolean("Cargo Arm Down Limit Switch: ", RobotMap.cargoArmLimitSwitchDown.get());
        // // SmartDashboard.putNumber("Cargo Arm Output Current: ", RobotMap.cargoArmTalon.getOutputCurrent());
        // // SmartDashboard.putNumber("Cargo Arm Distance PID Setpoint Please: ", RobotMap.cargoArm.distancePID.getSetpoint());
        //   SmartDashboard.putNumber("Cargo Arm Failsafe TN: ", RobotMap.cargoArm.failsafeTN.getOutput());
        // // SmartDashboard.putNumber("Cargo Arm Error: ", RobotMap.cargoArm.distancePID.getError());
        // // SmartDashboard.putNumber("Arm Ramped Setpoint:", RobotMap.cargoArm.distanceSetpointRampedTN.getOutput());
        // // SmartDashboard.putNumber("up ramp arm:", ConstantsIO.armDistanceSetpointUpRamp);
        //      SmartDashboard.putNumber("Cargo Arm output current", RobotMap.cargoArmTalon.getOutputCurrent());
        //     // SmartDashboard.putNumber("drive steering", DriveWithControllers.
        // // SmartDashboard.putNumber("Cargo Arm Distance Output TN ", RobotMap.cargoArm.distanceOutputTN.getOutput());
        // // SmartDashboard.putNumber("Cargo Arm Ramped Distance Setpoint TN: ", RobotMap.cargoArm.distanceSetpointRampedTN.getOutput());
        // // SmartDashboard.putNumber("Cargo Arm Ramped TN: ", RobotMap.cargoArm.distanceSetpointRampedTN.getOutput());
        // // SmartDashboard.putNumber("Cargo Roller Current", RobotMap.cargoRollersTalon.getOutputCurrent());
        // // SmartDashboard.putBoolean("Cargo Intaken: ", RobotMap.cargoRollers.intaken);
        // // SmartDashboard.putNumber("Drive Train Distance Setpoint: ", RobotMap.driveTrain.distanceSetpointTN.pidGet());
        // // SmartDashboard.putNumber("Drive Train Angle Setpoint: ", RobotMap.driveTrain.angleSetpointTN.pidGet());
        // // SmartDashboard.putNumber("DT Control Point X: ", DriveTrain.generateSandstormControlPoints(true)[0].getX());
        // // SmartDashboard.putNumber("DT Control Point Y: ", DriveTrain.generateSandstormControlPoints(true)[0].getY());
        // // SmartDashboard.putNumber("DT Endpoint X: ", DriveTrain.getSandstormEndpoint(true).getX());
        // // SmartDashboard.putNumber("DT Endpoint Y: ", DriveTrain.getSandstormEndpoint(true).getY());
        // // SmartDashboard.putBoolean("Drive Train Distance PID Enabled: ", RobotMap.driveTrain.distancePID.isEnabled());
        // // SmartDashboard.putBoolean("Drive Train Angle PID Enabled: ", RobotMap.driveTrain.anglePID.isEnabled());
        // // SmartDashboard.putNumber("Drive Train Talon Current: ", RobotMap.driveLeftTalon1.getOutputCurrent());
        // // SmartDashboard.putNumber("Drive Train Distance Output: ", RobotMap.driveTrain.distanceOutputTN.getOutput());
        // // SmartDashboard.putNumber("Drive Train Velocity Output: ", RobotMap.driveTrain.velocityOutputTN.getOutput());
        // // SmartDashboard.putNumber("Drive Train Velocity PID Setpoint: ", RobotMap.driveTrain.velocityPID.getSetpoint());
        // // SmartDashboard.putNumber("Drive Train Velocity PID Source: ", RobotMap.driveTrain.velocityPIDSource.pidGet());
        // // SmartDashboard.putNumber("Drive Train Distance Output TN: ", RobotMap.driveTrain.distanceOutputTN.getOutput());
        // // SmartDashboard.putNumber("Drive Train Distance Error: ", RobotMap.driveTrain.distancePID.getError());
        // // SmartDashboard.putNumber("Drive Train Velocity Error: ", RobotMap.driveTrain.velocityPID.getError());
        // // SmartDashboard.putNumber("DriveTrain Distance Source: ", RobotMap.driveTrain.distancePIDSource.pidGet());
        // SmartDashboard.putNumber("Drive Train Left Encoder Wrapper Dist: ", RobotMap.driveLeftEncoderWrapperDistance.pidGet());
        // SmartDashboard.putNumber("Drive Train Right Encoder Wrapper Dist: ", RobotMap.driveRightEncoderWrapperDistance.pidGet());
        // // SmartDashboard.putNumber("Drive Train Ang Vel Error: ", RobotMap.driveTrain.angVelPID.getError());
        // // SmartDashboard.putNumber("Drive Train Angle Error: ", RobotMap.driveTrain.anglePID.getError());
        // // SmartDashboard.putNumber("Cargo Arm Talon Output", RobotMap.cargoArmTalon.getMotorOutputPercent());
        // // SmartDashboard.putNumber("Drive Train Ang Vel Output", RobotMap.driveTrain.angVelOutputTN.getOutput());
        // // SmartDashboard.putNumber("kP Ang Vel", ConstantsIO.kP_DriveAngVel);
        // // SmartDashboard.putNumber("kP Ang Vel public version", RobotMap.driveTrain.angVelPID.kP);
        // // SmartDashboard.putNumber("Gyro Value:", RobotMap.gyroAngleWrapper.pidGet());
        // // SmartDashboard.putBoolean("Ang Vel Enabled: ", RobotMap.driveTrain.angVelPID.isEnabled());
        // SmartDashboard.putNumber("Drive Talon Left 1 Current:", RobotMap.driveLeftTalon1.getOutputCurrent());
        // SmartDashboard.putNumber("Drive Talon Left 2 Current:", RobotMap.driveLeftTalon2.getOutputCurrent());
        // SmartDashboard.putNumber("Drive Talon Left 3 Current:", RobotMap.driveLeftTalon3.getOutputCurrent());
        // SmartDashboard.putNumber("Drive Talon Left 4 Current:", RobotMap.driveLeftTalon4.getOutputCurrent());
        // SmartDashboard.putNumber("Drive Talon Right 1 Current:", RobotMap.driveRightTalon1.getOutputCurrent());
        // SmartDashboard.putNumber("Drive Talon Right 2 Current:", RobotMap.driveRightTalon2.getOutputCurrent());
        // SmartDashboard.putNumber("Drive Talon Right 3 Current:", RobotMap.driveRightTalon3.getOutputCurrent());
        // SmartDashboard.putNumber("Drive Talon Right 4 Current:", RobotMap.driveRightTalon4.getOutputCurrent());
        
        // SmartDashboard.putNumber("throttle", OI.getDriveThrottle());
        // SmartDashboard.putNumber("steering", OI.getDriveSteering());
        // SmartDashboard.putBoolean("angle is enabled?", RobotMap.driveTrain.anglePID.isEnabled());


        // // SmartDashboard.putBoolean("Drive Train Velocity Enabled: ", RobotMap.driveTrain.velocityPID.isEnabled());
        // // SmartDashboard.putNumber("Distance Output PID Source: ", RobotMap.cargoArm.distanceOutputPIDSource.pidGet());
        // // SmartDashboard.putBoolean("Lift Up: ", RobotMap.liftSolenoidOut.get());
        // // SmartDashboard.putNumber("Suraj RYStick output: ", OI.getArmManual());
        // // SmartDashboard.putNumber("Hatch Intake Rollers Current", RobotMap.hatchRollersTalon.getOutputCurrent());
        // // SmartDashboard.putNumber("Cargo Arm Failsafe TN: ", RobotMap.cargoArm.failsafeTN.getOutput());
        
        // SmartDashboard.putNumber("Drivetrain Left Velocity: ", RobotMap.driveLeftEncoderWrapperRate.pidGet());

        // SmartDashboard.putNumber("Drivetrain Velocity Setpoint", RobotMap.driveTrain.teleopSetpointLeftTN.getOutput());

        // SmartDashboard.putNumber("Scaled Error: ", RobotMap.driveTrain.teleopSetpointLeftRamp.scaledError);
      
    }
}