package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * Makes a button from a trigger or joystick.
 */
public class TriggerButton extends JoystickButton {

    private double threshold;
    private GenericHID joystick;
    private int port;
    private boolean bidirectional;

    /**
     * 
     * @param joystick The GenericHID object that has the button (e.g. Joystick, KinectStick,
   *                     etc)
     * @param port The port number of the joystick for the button.
     * @param threshold The minimum value the joystick must return to trigger a button.
     */
    public TriggerButton(GenericHID joystick, int port, double threshold) {
        super(joystick, port);
        this.threshold = threshold;
        this.joystick = joystick;
        this.port = port;
        this.bidirectional = false;
    }

    public TriggerButton(GenericHID joystick, int port, double threshold, boolean bidirectional) {
        super(joystick, port);
        this.threshold = threshold;
        this.joystick = joystick;
        this.port = port;
        this.bidirectional = bidirectional;
    } 

    /**
     * Returns true if the joystick value is greater than the given threshold.
     */
    @Override
    public boolean get() {
        if (threshold < 0) {
            return this.joystick.getRawAxis(this.port) <= threshold;
        }
        if (bidirectional) {
            return this.joystick.getRawAxis(this.port) >= threshold || this.joystick.getRawAxis(this.port) <= -threshold;
        }
        return this.joystick.getRawAxis(this.port) >= threshold;
    }
}
