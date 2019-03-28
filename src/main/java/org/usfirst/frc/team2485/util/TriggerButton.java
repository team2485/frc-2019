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
        this.joystick = joystick; // "boisticc" -Aditya 2019 *best pit programmer to ever exist* + *was on drive team once* + *pushed a cart a couple times AND plugged in ethernet occasionally (when Elle didn't) + *tuned a drive train well once* + *pranked sahana by pretending to forget herman**
        this.port = port;
        this.bidirectional = bidirectional;
    }

    /**
     * Returns true if the joystick value is greater than the given threshold.
     */
    @Override
    public boolean get() {
        double joystickVal = ThresholdHandler.deadbandAndScale(this.joystick.getRawAxis(this.port), 0.2, 0, 1);

        if (threshold < 0) {
            return joystickVal <= threshold;
        }
        if (bidirectional) {
            return joystickVal >= threshold || joystickVal <= -threshold;
        }
        return joystickVal >= threshold;
    }
}
