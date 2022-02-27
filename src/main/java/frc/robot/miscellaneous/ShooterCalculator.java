// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.miscellaneous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class ShooterCalculator {

    private static double mainShooterDiameter = 4.0;
    private static double backspinShooterDiameter = 3.0;
    private static double desiredCargoRPM = 120.0;

    public static double calculateZero (double a) {
        return a-a;
    }

    public static double calculateBackspinRPM (double mainShooterRPM) {

        /* This method should take as input the current RPM of the main shooter
        *  wheel as well as the CONSTANTS for shooter diameters and desired
        *  cargo RPM. The method will return the _desired_ RPM of the backspin
        *  motor. 
        *
        *  Please Note: This method assumes that RPM is the correct metric to use.
        *  If we need something other than RPM (like motor power) we'll need to
        *  update the calculation to tranlate this to another measure.
        * 
        */

        double desiredBackspinRPM;

        // Check to see if there is a desiredCargoRPM value in SmartDashboard
        if(SmartDashboard.getNumber("cargo.desiredRPM", 0) == 0) {
          // The number must not exist. Let's go ahead and create it.
          SmartDashboard.putNumber("cargo.desiredRPM", desiredCargoRPM);
        }
        desiredCargoRPM = SmartDashboard.getNumber("cargo.desiredRPM", desiredCargoRPM);

        desiredBackspinRPM = (mainShooterRPM - (2 * 2.5 * desiredCargoRPM)) * (mainShooterDiameter/backspinShooterDiameter);

        return desiredBackspinRPM;

    }




}
