package org.usfirst.frc.team2485.util;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * A class that allows a single encoder to be used as a PIDSource in both
 * rate mode and displacement mode simultaneously.  
 * @author Jeremy McCulloch
 */
public class EncoderWrapperRateAndDistance implements PIDSource {
	private Encoder encoder;
	private PIDSourceType pidSource;
	private double gearRatio = 16.0/62;
	public EncoderWrapperRateAndDistance(Encoder encoder, PIDSourceType pidSource) {
		this.encoder = encoder;
		this.pidSource = pidSource;
	}
	
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		this.pidSource = pidSource;
	}
	@Override
	public PIDSourceType getPIDSourceType() {
		return pidSource;
	}
	@Override
	public double pidGet() {
		if (pidSource == PIDSourceType.kDisplacement) {
			return (encoder.getDistance() * gearRatio);
		} else {
			return (encoder.getRate() * gearRatio);
		}
	}

	public void setGearRatio(double gearRatio) {
		this.gearRatio = gearRatio;
	}
}
