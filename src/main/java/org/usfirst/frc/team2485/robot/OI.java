/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2485.robot;
import org.usfirst.frc.team2485.robot.commandGroups.CargoIntake;
import org.usfirst.frc.team2485.robot.commandGroups.LoadingStationIntake;
import org.usfirst.frc.team2485.robot.commandGroups.PlaceHatch;
import org.usfirst.frc.team2485.robot.commands.CancelCommand;
import org.usfirst.frc.team2485.robot.commands.CargoArmWithControllers;
import org.usfirst.frc.team2485.robot.commands.CargoRollersIntake;
import org.usfirst.frc.team2485.robot.commands.EjectCargo;
import org.usfirst.frc.team2485.robot.commands.Hook;
import org.usfirst.frc.team2485.robot.commands.Lift;
import org.usfirst.frc.team2485.robot.commands.Pushers;
import org.usfirst.frc.team2485.robot.commands.SetArmPosition;
import org.usfirst.frc.team2485.robot.commands.SetElevatorPosition;
import org.usfirst.frc.team2485.robot.commands.SetRollers;
import org.usfirst.frc.team2485.robot.commands.SetRollersCurrent;
import org.usfirst.frc.team2485.robot.commands.Slide;
import org.usfirst.frc.team2485.robot.subsystems.CargoArm;
import org.usfirst.frc.team2485.robot.subsystems.Elevator.ElevatorLevel;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.ThresholdHandler;
import org.usfirst.frc.team2485.util.TriggerButton;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public static Joystick jacket;
	public static Joystick suraj;
	public static Joystick jacketBackup;
	public static Joystick surajBackup;

	//Ports
	public static final int XBOX_A_PORT = 1;
	public static final int XBOX_B_PORT = 2;
	public static final int XBOX_X_PORT = 3;
	public static final int XBOX_Y_PORT = 4;
	public static final int XBOX_LBUMPER_PORT = 5;
	public static final int XBOX_RBUMPER_PORT = 6; 
	public static final int XBOX_BACK_BUTTON = 7;
	public static final int XBOX_START_BUTTON = 8;
	public static final int XBOX_LSTICK_BUTTON_PORT = 9;  
	public static final int XBOX_RSTICK_BUTTON_PORT = 10;  
	public static final int XBOX_XBOX_PORT = 11;  
	public static final int XBOX_UP_PORT = 12;  
	public static final int XBOX_DOWN_PORT = 13;  
	public static final int XBOX_LEFT_PORT = 14;
	public static final int XBOX_RIGHT_PORT = 15;
	
	public static final int XBOX_LXJOSYSTICK_PORT = 0;
	public static final int XBOX_LYJOYSTICK_PORT = 1;
	public static final int XBOX_LTRIGGER_PORT = 2;
	public static final int XBOX_RTRIGGER_PORT = 3;
	public static final int XBOX_RXJOYSTICK_PORT = 4;
	public static final int XBOX_RYJOYSTICK_PORT = 5;

	public static final int G13_BUTTON_PORT1 = 0;
	public static final int G13_BUTTON_PORT2 = 1;

	
	//Joystick Buttons
	public static JoystickButton JACKET_UP;
	public static JoystickButton JACKET_DOWN;
	public static JoystickButton JACKET_LEFT;
	public static JoystickButton JACKET_RIGHT;
	public static JoystickButton JACKET_A;
	public static JoystickButton JACKET_B;
	public static JoystickButton JACKET_X;
	public static JoystickButton JACKET_Y;
	public static JoystickButton JACKET_LBUMPER;
	public static JoystickButton JACKET_RBUMPER;
	public static JoystickButton JACKET_XBOX;
	public static JoystickButton JACKET_START;
	public static JoystickButton JACKET_RSTICK_BUTTON;
	public static JoystickButton JACKET_LSTICK_BUTTON;
	public static JoystickButton JACKET_BACK_BUTTON;
	public static JoystickButton JACKET_LTRIGGER_BUTTON;
	public static JoystickButton JACKET_RTRIGGER_BUTTON;

	public static JoystickButton JACKET_A_BACKUP;
	public static JoystickButton JACKET_RBUMPER_BACKUP;

	
	public static JoystickButton SURAJ_UP;
	public static JoystickButton SURAJ_DOWN;
	public static JoystickButton SURAJ_LEFT;
	public static JoystickButton SURAJ_RIGHT;
	public static JoystickButton SURAJ_A;
	public static JoystickButton SURAJ_B;
	public static JoystickButton SURAJ_X;
	public static JoystickButton SURAJ_Y;
	public static JoystickButton SURAJ_LBUMPER;
	public static JoystickButton SURAJ_RBUMPER;
	public static JoystickButton SURAJ_XBOX;
	public static JoystickButton SURAJ_LSTICK_BUTTON;
	public static JoystickButton SURAJ_RSTICK_BUTTON;
	public static JoystickButton SURAJ_START_BUTTON;
	public static JoystickButton SURAJ_BACK_BUTTON;

	public static JoystickButton SURAJ_A_BACKUP;
	public static JoystickButton SURAJ_B_BACKUP;
	public static JoystickButton SURAJ_X_BACKUP;
	public static JoystickButton SURAJ_Y_BACKUP;
	public static JoystickButton SURAJ_START_BUTTON_BACKUP;
	public static JoystickButton SURAJ_LBUMPER_BACKUP;
	public static JoystickButton SURAJ_RBUMPER_BACKUP;
	


	public static TriggerButton SURAJ_LTRIGGER_BUTTON;
	




	public static void init(){
		
		jacket = new Joystick(0);
		suraj = new Joystick(1);
		jacketBackup = new Joystick(2);
		surajBackup = new Joystick(3);
				
		JACKET_UP = new JoystickButton(jacket, XBOX_UP_PORT);
		JACKET_DOWN = new JoystickButton(jacket, XBOX_DOWN_PORT);
		JACKET_LEFT = new JoystickButton(jacket, XBOX_LEFT_PORT);
		JACKET_RIGHT = new JoystickButton(jacket, XBOX_RIGHT_PORT);
		JACKET_A = new JoystickButton(jacket, XBOX_A_PORT);
		JACKET_B = new JoystickButton(jacket, XBOX_B_PORT);
		JACKET_X = new JoystickButton(jacket, XBOX_X_PORT);
		JACKET_Y = new JoystickButton(jacket, XBOX_Y_PORT);
		JACKET_LBUMPER = new JoystickButton(jacket, XBOX_LBUMPER_PORT);
		JACKET_RBUMPER = new JoystickButton(jacket, XBOX_RBUMPER_PORT);
		JACKET_XBOX = new JoystickButton(jacket, XBOX_XBOX_PORT);
		JACKET_START = new JoystickButton(jacket, XBOX_START_BUTTON);
		JACKET_BACK_BUTTON = new JoystickButton(jacket, XBOX_BACK_BUTTON);
		JACKET_RSTICK_BUTTON = new JoystickButton(jacket, XBOX_RSTICK_BUTTON_PORT);
		JACKET_LSTICK_BUTTON = new JoystickButton(jacket, XBOX_LSTICK_BUTTON_PORT);

		JACKET_A_BACKUP = new JoystickButton(jacketBackup, XBOX_A_PORT);
		JACKET_RBUMPER_BACKUP = new JoystickButton(jacketBackup, XBOX_RBUMPER_PORT);


		
		SURAJ_UP = new JoystickButton(suraj, XBOX_UP_PORT);
		SURAJ_DOWN = new JoystickButton(suraj, XBOX_DOWN_PORT);
		SURAJ_LEFT = new JoystickButton(suraj, XBOX_LEFT_PORT);
		SURAJ_RIGHT = new JoystickButton(suraj, XBOX_RIGHT_PORT);
		SURAJ_A = new JoystickButton(suraj, XBOX_A_PORT);
		SURAJ_B = new JoystickButton(suraj, XBOX_B_PORT);
		SURAJ_X = new JoystickButton(suraj, XBOX_X_PORT);
		SURAJ_Y = new JoystickButton(suraj, XBOX_Y_PORT);
		SURAJ_LBUMPER = new JoystickButton(suraj, XBOX_LBUMPER_PORT);
		SURAJ_RBUMPER = new JoystickButton(suraj, XBOX_RBUMPER_PORT);
		SURAJ_XBOX = new JoystickButton(suraj, XBOX_XBOX_PORT);
		SURAJ_START_BUTTON = new JoystickButton(suraj, XBOX_START_BUTTON);
		SURAJ_BACK_BUTTON = new JoystickButton(suraj, XBOX_BACK_BUTTON);
		SURAJ_LSTICK_BUTTON = new JoystickButton(suraj, XBOX_LSTICK_BUTTON_PORT);
		SURAJ_RSTICK_BUTTON = new JoystickButton(suraj, XBOX_RSTICK_BUTTON_PORT);

		SURAJ_LTRIGGER_BUTTON = new TriggerButton(suraj, XBOX_LTRIGGER_PORT, 0.2);
		

		SURAJ_A_BACKUP = new JoystickButton(surajBackup, XBOX_A_PORT);
		SURAJ_B_BACKUP = new JoystickButton(surajBackup, XBOX_B_PORT);
		SURAJ_X_BACKUP = new JoystickButton(surajBackup, XBOX_X_PORT);
		SURAJ_Y_BACKUP = new JoystickButton(surajBackup, XBOX_Y_PORT);

		SURAJ_LBUMPER_BACKUP = new JoystickButton(surajBackup, XBOX_LBUMPER_PORT);
		SURAJ_RBUMPER_BACKUP = new JoystickButton(surajBackup, XBOX_RBUMPER_PORT);

		

		// SURAJ_START_BUTTON_BACKUP = new JoystickButton(surajBackup, XBOX_START_BUTTON);
		// SURAJ_LTRIGGER_BUTTON_BACKUP = new TriggerButton(surajBackup, XBOX_LTRIGGER_PORT, 0.2);



		// SURAJ_A.whenPressed(new Lift(false));
		// SURAJ_B.whenPressed(new Hook(true));
		// SURAJ_X.whenPressed(new Pushers(true));
		// SURAJ_Y.whenPressed(new Slide(true));

		CommandGroup placeHatch = new PlaceHatch();
		SURAJ_LBUMPER.whenPressed(placeHatch);
		//SURAJ_LBUMPER.whenPressed(new CancelCommand(Robot.auto));
		// SURAJ_LBUMPER_BACKUP.whenPressed(placeHatch);
		// SURAJ_LBUMPER_BACKUP.whenPressed(new CancelCommand(Robot.auto));



		
		// SURAJ_LYJOYSTICK_BACKUP.whenPressed(new CancelCommand(placeHatch));
		// SURAJ_LYJOYSTICK_BACKUP.whenPressed(new CancelCommand(Robot.auto));
		// SURAJ_RBUMPER.whenPressed(new Hook(false));
		// SURAJ_START_BUTTON.whenPressed(new Pushers(false));
		SURAJ_RBUMPER.whenPressed(new LoadingStationIntake());
		// SURAJ_RBUMPER_BACKUP.whenPressed(new LoadingStationIntake());
		// SURAJ_RBUMPER.whenPressed(new CancelCommand(Robot.auto));
		// SURAJ_RBUMPER.whenPressed(new CancelCommand()

		JACKET_RBUMPER.whenPressed(new Hook(false));
		// JACKET_RBUMPER_BACKUP.whenPressed(new Hook(false));
		// JACKET_RBUMPER.whenPressed(new 
		// CancelCommand(Robot.auto));

		Command floor = new SetElevatorPosition(ElevatorLevel.FLOOR);
		SURAJ_A.whenPressed(floor);
		// SURAJ_A.whenPressed(new CancelCommand(Robot.auto));
		SURAJ_A.whenReleased(new CancelCommand(floor));
		// SURAJ_A_BACKUP.whenPressed(floor);
		// SURAJ_A_BACKUP.whenReleased(new CancelCommand(floor));
		
		Command rocketLevelOne = new SetElevatorPosition(ElevatorLevel.ROCKET_LEVEL_ONE); 
		SURAJ_X.whenPressed(rocketLevelOne);
		// SURAJ_X.whenPressed(new CancelCommand(Robot.auto));
		SURAJ_X.whenReleased(new CancelCommand(rocketLevelOne));
		// SURAJ_X_BACKUP.whenPressed(rocketLevelOne);
		// SURAJ_X_BACKUP.whenReleased(new CancelCommand(rocketLevelOne));

		Command rocketLevelTwo = new SetElevatorPosition(ElevatorLevel.ROCKET_LEVEL_TWO); 
		SURAJ_B.whenPressed(rocketLevelTwo);
		// SURAJ_B.whenPressed(new CancelCommand(Robot.auto));
		SURAJ_B.whenReleased(new CancelCommand(rocketLevelTwo));
		// SURAJ_B_BACKUP.whenPressed(rocketLevelTwo);
		// SURAJ_B_BACKUP.whenReleased(new CancelCommand(rocketLevelTwo));

		
		Command rocketLevelThree = new SetElevatorPosition(ElevatorLevel.ROCKET_LEVEL_THREE); 
		SURAJ_Y.whenPressed(rocketLevelThree);
		// SURAJ_Y.whenPressed(new CancelCommand(Robot.auto));
		SURAJ_Y.whenReleased(new CancelCommand(rocketLevelThree));
		// SURAJ_Y_BACKUP.whenPressed(rocketLevelThree);
		// SURAJ_Y_BACKUP.whenReleased(new CancelCommand(rocketLevelThree));

		
		
		// SURAJ_A.whenPressed(new SetElevatorPosition(ElevatorLevel.FLOOR));
		// // SURAJ_A.whenReleased(new CancelCommand(floor));
		
		// SURAJ_X.whenPressed(new SetElevatorPosition(ElevatorLevel.ROCKET_LEVEL_ONE));
		// SURAJ_B.whenPressed(new SetElevatorPosition(ElevatorLevel.ROCKET_LEVEL_TWO));
		// SURAJ_Y.whenPressed(new SetElevatorPosition(ElevatorLevel.ROCKET_LEVEL_THREE));
		
		
		// SURAJ_RYJOYSTICK_BACKUP.whenPressed(new CargoArmWithControllers());
		//SURAJ_RYJOYSTICK.whenPressed(new CancelCommand(Robot.auto));

		
		SURAJ_START_BUTTON.whenPressed(new Lift(true));
		// SURAJ_START_BUTTON_BACKUP.whenPressed(new Lift(true));
		// SURAJ_START_BUTTON.whenPressed(new CancelCommand(Robot.auto));
		SURAJ_LTRIGGER_BUTTON.whenPressed(
			new SetRollers(-0.4));
		// SURAJ_LTRIGGER_BUTTON.whenPressed(new C/\@
		// ancelCommand(Robot.auto));
		SURAJ_LTRIGGER_BUTTON.whenReleased(new SetRollers(0));

		// SURAJ_LTRIGGER_BUTTON_BACKUP.whenPressed(new SetRollers(-0.4));
		// SURAJ_LTRIGGER_BUTTON_BACKUP.whenReleased(new SetRollers(0));

		// JACKET_A.whenPressed(new SetRollers(0.4));
		// JACKET_B.whenPressed(new SetRollers(0));
		// JACKET_Y.whenPressed(new SetRollers(-0.4));
		// JACKET_RBUMPER.whenPressed(new CargoRollersIntake(0.4));
	
		
		JACKET_A.whenPressed(new CargoIntake(0.4));
		JACKET_A.whenPressed(new SetRollersCurrent());
		SURAJ_BACK_BUTTON.whenPressed(new Hook(true));
		// JACKET_A_BACKUP.whenPressed(new CargoIntake(0.4));
		// JACKET_A.whenPressed(new CancelCommand(Robot.auto));

		// JACKET_LTRIGGER_BUTTON.whenPressed(new CancelCommand(Robot.auto));
		// JACKET_RTRIGGER_BUTTON.whenPressed(new CancelCommand(Robot.auto));
		//temporary
	
		//SURAJ_BACK_BUTTON.whenPressed(new Lift(false));

	}

	public static boolean getQuickTurn() {
		return OI.jacket.getRawButton(OI.XBOX_X_PORT);
	}

	public static double getDriveThrottle() {
		return ThresholdHandler.deadbandAndScale(OI.jacket.getRawAxis(OI.XBOX_RTRIGGER_PORT), 0.2, 0, ConstantsIO.driveTrainIMax/2) 
		- ThresholdHandler.deadbandAndScale(OI.jacket.getRawAxis(OI.XBOX_LTRIGGER_PORT), 0.2, 0, ConstantsIO.driveTrainIMax/2);
	}

	public static double getDriveSteering() {
		return ThresholdHandler.deadbandAndScale(OI.jacket.getRawAxis(OI.XBOX_LXJOSYSTICK_PORT), 0.2, 0, ConstantsIO.driveTrainIMax/2);
	}

	public static double getArmManual() {
		return ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_RYJOYSTICK_PORT), 0.2, 0, 0.6);
	}

	public static double getElevatorManual() {
		return ThresholdHandler.deadbandAndScale(OI.suraj.getRawAxis(OI.XBOX_LYJOYSTICK_PORT), 0.2, 0, 0.6);
	}




}
