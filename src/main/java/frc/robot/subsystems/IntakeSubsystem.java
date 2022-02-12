// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class IntakeSubsystem extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = new Color(0.22, 0.43, 0.35);
  //Light on: private final Color kBlueTarget = new Color(0.18, 0.42, 0.40);
  //private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kRedTarget = new Color(0.65, 0.29, 0.06);
  //Light on: private final Color kRedTarget = new Color(0.38, 0.43, 0.19);
  //private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    m_colorMatcher.setConfidenceThreshold(0.95);
    m_colorMatcher.addColorMatch(kBlueTarget);
    //m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    //m_colorMatcher.addColorMatch(kYellowTarget);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Color detectedColor = m_colorSensor.getColor();
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchColor(detectedColor);
    if (match == null) {
      colorString = "null";

    } else if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else {
      colorString = match.toString();
    }


    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    if (match == null) {
      SmartDashboard.putNumber("Confidence", -1.0);
    } else {
      SmartDashboard.putNumber("Confidence", match.confidence);
    }
    SmartDashboard.putString("Detected Color", colorString);
  }
}