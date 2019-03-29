package org.usfirst.frc.team2485.robot.commands;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.subsystems.Elevator;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.EncoderWrapperRateAndDistance;
import org.usfirst.frc.team2485.util.ThresholdHandler;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

public class ElevatorWithControllers extends Command {
    public static double holdPosition = 0.0;
    private double elevatorSpikeCurrent = 10.0;
    private boolean spiking;
    private long startSpikeTime;
    private int spikeTime = 20000;
    public static double power = 0.0;
    private boolean first;
    private int fullPowerTime = 500;
    public static boolean encoderMovement = true;
    public static boolean manualMovement = false;
    private long startEncoderLossTime;

    public ElevatorWithControllers() {
        this.setInterruptible(true);
        this.requires(RobotMap.elevator);
        this.first = false;
    }

    @Override
    protected void initialize() {
        if(!manualMovement) {
            holdPosition = RobotMap.elevator.lastLevel.getPosition();
            RobotMap.elevator.distanceSetpointTN.setOutput(power);
            RobotMap.elevator.enablePID(true);
        }
    }

    @Override
    protected void execute() {
        // System.out.println("EncoderMovement: " + encoderMovement);
        System.out.println("Manual Movement: " + manualMovement);
        if(!manualMovement) {
            boolean zero = ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_LYJOYSTICK_PORT), 0.2, 0.0, 1.0) == 0.0;
            if (!zero) {
                power -= ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_LYJOYSTICK_PORT), 0.2, 0.0, 1.0) * 1.2;
                this.first = true;
            } else if (this.first) {
                power = RobotMap.elevatorEncoderWrapperDistance.pidGet();
                this.first = false;
            }
            RobotMap.elevator.distanceSetpointTN.setOutput(power);
            if (RobotMap.elevatorEncoder.pidGet() <= 0.0) {
                RobotMap.elevator.distancePID.resetIntegrator();
            }
            if (RobotMap.elevatorTalon1.getOutputCurrent() >= this.elevatorSpikeCurrent && !RobotMap.elevator.distancePID.isOnTarget()) {
                if (!this.spiking) {
                    this.startSpikeTime = System.currentTimeMillis();
                    this.spiking = true;
                }
            } else {
                this.spiking = false;
                Elevator.enableFailsafe = false;
            }
            if (this.spiking && System.currentTimeMillis() - this.startSpikeTime >= (long)this.spikeTime) {
                Elevator.enableFailsafe = true;
            }
            
            if(RobotMap.elevator.distanceOutputTN.pidGet() > 5 && RobotMap.elevatorEncoderWrapperDistance.pidGet() == 0 && encoderMovement) {
                RobotMap.elevator.enablePID(false);
                encoderMovement = false;
                startEncoderLossTime = System.currentTimeMillis();
                manualMovement = true;
                System.out.println("Turning on 1");
            } 
            if(!encoderMovement) {
                if(RobotMap.elevatorEncoderWrapperDistance.pidGet() != 0) {
                    encoderMovement = true;
                } 
                if(System.currentTimeMillis() - startEncoderLossTime >= fullPowerTime && RobotMap.elevator.distancePID.getSetpoint() >= ElevatorLevel.ROCKET_LEVEL_ONE.getPosition()) {
                    System.out.println("Turning on 2");
                    manualMovement = true;
                }
            }
        } else {
            RobotMap.elevator.enablePID(false);
            RobotMap.elevator.motorSetter.disable();
            RobotMap.elevator.elevatorManual(-ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_LYJOYSTICK_PORT), 0.2, 0, 1));
        }
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}