package org.usfirst.frc3620.misc;

import edu.wpi.first.wpilibj.Joystick;

public class AnalogJoystickButton extends AnalogValueButton {

    public AnalogJoystickButton (Joystick joystick, int axis) {
        this (joystick, axis, 0.2);
    }

    public AnalogJoystickButton (Joystick joystick, int axis, double threshold) {
        super(() -> joystick.getRawAxis(axis), threshold);
    }

}
