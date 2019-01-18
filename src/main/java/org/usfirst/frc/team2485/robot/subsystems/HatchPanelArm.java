package org.usfirst.frc.team2485.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team2485.util.WarlordsPIDController;
import org.usfirst.frc.team2485.util.WarlordsPIDSource;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.util.MotorSetter;
import org.usfirst.frc.team2485.util.ConstantsIO;

public class HatchPanelArm extends Subsystem {

    private static final double MAX_ANG = Math.PI; //change
    private static final double MIN_ANG = -Math.PI; //change
    private static final double MAX_ANGVEL = 10; //change
    private static final double MAX_CURR = 10; //change

    private WarlordsPIDController anglePID, angVelPID;

    private PIDSourceWrapper anglePIDSource, angVelPIDSource, currentPIDSource;

    private TransferNode angleTN, angVelTN;

	private MotorSetter motorSetter;

    public HatchPanelArm() {

        anglePID = new WarlordsPIDController();
        angVelPID = new WarlordsPIDController();

        anglePIDSource = new PIDSourceWrapper();
        angVelPIDSource = new PIDSourceWrapper();
        currentPIDSource = new PIDSourceWrapper();

        angleTN = new TransferNode(0);
        angVelTN = new TransferNode(0);

        motorSetter = new MotorSetter();

        updateConstants();

        anglePIDSource.setPidSource(() -> {
            return RobotMap.hatchPanelArmEncoderWrapperDistance.pidGet();
        });

        anglePID.setSources(anglePIDSource);
        anglePID.setInputRange(MIN_ANG, MAX_ANG);
        anglePID.setOutputRange(-MAX_ANGVEL, MAX_ANGVEL);
        anglePID.setOutputs(angleTN);

        angVelPIDSource.setPidSource(() -> {
            return RobotMap.hatchPanelArmEncoderWrapperRate.pidGet();
        });

        angVelPID.setSetpointSource(angleTN);
        angVelPID.setSources(angVelPIDSource);
        angVelPID.setOutputs(angVelTN);
        angVelPID.setOutputRange(-MAX_CURR, MAX_CURR);

        currentPIDSource.setPidSource(() -> {
            return angVelTN.pidGet();
        });

        motorSetter.setSources(currentPIDSource);
        motorSetter.setOutputs(RobotMap.hatchPanelArmSC);

    }

    public void updateConstants(){
        anglePID.setPID(ConstantsIO.kP_WristAng, 0, 0);
        angVelPID.setPID(ConstantsIO.kP_WristAngVel, ConstantsIO.kI_WristAngVel, 0, ConstantsIO.kF_WristAngVel);
        
    }

    public void enablePID(boolean PID) {
        if(PID){
            anglePID.enable();
            angVelPID.enable();
        } else {
            anglePID.disable();
            angVelPID.disable();
        }
    }

    @Override
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

}