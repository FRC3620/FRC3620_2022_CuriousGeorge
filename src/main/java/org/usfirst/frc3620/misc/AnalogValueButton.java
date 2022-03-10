package org.usfirst.frc3620.misc;

import edu.wpi.first.wpilibj2.command.button.*;

public class AnalogValueButton extends Button {
	AnalogValueProvider provider;
	double threshold;

	public AnalogValueButton() {
		this (null, 0.2);
	}

	public AnalogValueButton(AnalogValueProvider provider, double threshold) {
		super();
		this.provider = provider;
		this.threshold = threshold;
	}

	@Override
	public boolean get() {
		if (threshold > 0) {
			return provider.get() > threshold;
		} else {
			return provider.get() < threshold;
		}
	}

}
