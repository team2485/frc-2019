package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.PIDOutput;

/**
 * Generic class to ramp an output (velocity, voltage, current, etc). Has unique up and down rates. 
 * @author Jeremy McCulloch
 * Quadratic:
 * @author Ian Lillie
 * @author Mark Rifkin
 */
public class RampRate extends WarlordsControlSystem {
	
	private double lastValue, upRampRate, downRampRate; 
	private double upRampRateA, upRampRateB, downRampRateA, downRampRateB;
	private boolean isQuadratic;

	private double maxErrorToDesired;
	public double scaledError;
	
	public RampRate() { 
		lastValue = 0;
		this.isQuadratic = false;
	}

	public RampRate(boolean isQuadratic){
		this.isQuadratic = isQuadratic;
		this.lastValue = 0;
		this.maxErrorToDesired = 0;
	}	
	
	public void setRampRates(double upRampRate, double downRampRate) {
		this.upRampRate = upRampRate;
		this.downRampRate = downRampRate;
	}

	public void setRampRates(double upRampRateA, double upRampRateB, double downRampRateA, double downRampRateB) {
		this.upRampRateA = upRampRateA;
		this.upRampRateB = upRampRateB;
		this.downRampRateA = downRampRateA;
		this.downRampRateB = downRampRateB;
		
	}
	
	/**
	 * Used to get next value by moving from the last value toward the desired value, without exceeding the ramp rate
	 * @param desired target value
	 * @return actual value
	 */
	
	public double getNextValue(double desired) {
		if ((lastValue > 0 && desired < 0) || (lastValue < 0 && desired > 0)) {
			desired = 0; // makes sure desired and lastValue have the same sign to make math easy
		}
		//System.out.println("Desired: " + desired);
		double errorToDesired = Math.abs(desired - lastValue);


		if (errorToDesired > maxErrorToDesired) {
			maxErrorToDesired = errorToDesired;
		}

		scaledError = errorToDesired / maxErrorToDesired; //scaled error is now mapped between 0 and 1

		if(!isQuadratic) {
		
		if (Math.abs(desired) > Math.abs(lastValue)) {
			if (Math.abs(desired - lastValue) > upRampRate) {
				if (desired > 0) {
					lastValue += upRampRate;
				} else {
					lastValue -= upRampRate;
				}
			} else {
				lastValue = desired;
			}
		} else {
			if (Math.abs(desired - lastValue) > downRampRate) {
				if (lastValue > 0) {
					lastValue -= downRampRate;
				} else {
					lastValue += downRampRate;
				}
			} else {
				lastValue = desired;
			}
		}
		
		return lastValue;

	} else {
		// System.out.println("Scaled Error: " + scaledError);
		// System.out.println("Max Error: " + maxErrorToDesired);
		// System.out.println("Error: " + errorToDesired);

		if (Math.abs(desired) > Math.abs(lastValue)) {
			if (Math.abs(desired - lastValue) > upRampRateB + upRampRateA * scaledError) {
				if (desired > 0) {
					lastValue += upRampRateB + upRampRateA * scaledError;
				} else {
					lastValue -= upRampRateB + upRampRateA * scaledError;
				}
			} else {
				lastValue = desired;
			}
		} else {
			if (Math.abs(desired - lastValue) > downRampRateB + downRampRateA * scaledError) {
				if (lastValue > 0) {
					lastValue -= downRampRateB + downRampRateA / scaledError ;
				} else {
					lastValue += downRampRateB + downRampRateA / scaledError;
				}
			} else {
				lastValue = desired;
			}
		}
		
		return lastValue;
	}
}

	
	/**
	 * Used to immediately set the last value, potentially not following ramp rate
	 * @param lastValue new value to be treated as the last value
	 */
	public void setLastValue(double lastValue) {
		this.lastValue = lastValue;
	}
	
	/**
	 * @return isQuadratic
	 */
	public boolean isQuadratic() {
		return isQuadratic;
	}

	@Override
	protected void calculate() {
		double val = getNextValue(setpoint);
		for (PIDOutput out : outputs) {
			out.pidWrite(val);
		}
	}
	
	@Override
	public void disable() {
		setLastValue(0);
		setpoint = 0;
		super.disable();
	}
}
