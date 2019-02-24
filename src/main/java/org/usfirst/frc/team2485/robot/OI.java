/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2485.robot;
import org.usfirst.frc.team2485.robot.commands.Hook;
import org.usfirst.frc.team2485.robot.commands.Lift;
import org.usfirst.frc.team2485.robot.commands.Pushers;
import org.usfirst.frc.team2485.robot.commands.SetRollers;
import org.usfirst.frc.team2485.robot.commands.Slide;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	public static Joystick jacket;
	public static Joystick suraj;

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
	private static JoystickButton JACKET_BACK_BUTTON;

	
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

	//// CREATING BUTTONS
	// One type of button is a joystick button which is any button on a
	//// joystick.
	// You create one by telling it which joystick it's on and which button
	// number it is.
	// Joystick stick = new Joystick(port);
	// Button button = new JoystickButton(stick, buttonNumber);

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());

	public static void init(){
		
		jacket = new Joystick(0);
		suraj = new Joystick(1);
				
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



		SURAJ_A.whenPressed(new Lift(false));
		SURAJ_B.whenPressed(new Hook(true));
		SURAJ_X.whenPressed(new Pushers(true));
		SURAJ_Y.whenPressed(new Slide(true));

		SURAJ_LBUMPER.whenPressed(new Lift(true));
		SURAJ_RBUMPER.whenPressed(new Hook(false));
		SURAJ_START_BUTTON.whenPressed(new Pushers(false));
		SURAJ_BACK_BUTTON.whenPressed(new Slide(false));

		JACKET_A.whenPressed(new SetRollers(0.4));
		JACKET_B.whenPressed(new SetRollers(0));
		JACKET_Y.whenPressed(new SetRollers(-0.4));

	}
}
