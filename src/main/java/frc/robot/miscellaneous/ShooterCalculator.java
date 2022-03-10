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


    public static double calcHoodPosition(double distance) {
        double calcHoodPosition;
        if(distance < 12.5){
            calcHoodPosition = 129 + -9.9*distance + 0.36*distance*distance;
        } else if(distance < 17.5){
            calcHoodPosition = 74 + -0.5*distance + -0.04*distance*distance;
        } else {
            calcHoodPosition = 109 + -4.6*distance + 0.08*distance*distance;
        }
        return calcHoodPosition;
    }

    public static double calcMainRPM(double distance) {
        double calcMainRPM = 1260 + (44*distance);
        
        return calcMainRPM;
    }
    public static double calcDistanceFromHub(double targetY){
        double distance;


        if (targetY>0.461){
            distance = 20.05629 - 19.60707*(targetY) + 6.976088*(targetY*targetY);
            
        } else if(targetY>0.239) {
            distance = 26.38900 - 44.79796*(targetY) + 31.82205*(targetY*targetY);
        } else {
            distance = 27.98076 -63.49953 *(targetY) +82.20488 *(targetY*targetY);

        }


        return distance;



    }

}
