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


    public double calcHoodPosition(double cy) {
        double calcHoodPosition;
        if(cy < 224){
            calcHoodPosition = 3.73187317480733 + 0.0327847309136473*cy +-0.0000114726741759497*cy*cy;
            calcHoodPosition = calcHoodPosition + SmartDashboard.getNumber("manualHoodPosition", 5);
        } else if(cy < 336){
            calcHoodPosition = 3.85000000000 + 0.0369791667*cy + -0.0000325521*cy*cy;
        }else if(cy < 403){
            calcHoodPosition = -28.1396700696 + 0.2136292223*cy + -0.0002749411*cy*cy;
        } else {
            calcHoodPosition = -56.8299016952515 + 0.355106208706275*cy + -0.000449346405275719*cy*cy;
        }
        return 5.0 * calcHoodPosition;
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
