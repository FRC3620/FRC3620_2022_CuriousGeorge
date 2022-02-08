/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubSystem;

public class AutoDriveToCargoCommand extends CommandBase {

    private DriveSubsystem driveSubsystem;
    private VisionSubSystem visionSubsystem;

    private double initialPositionRightFront;
    private double initialPositionLeftFront;
    private double initialPositionRightBack;
    private double initialPositionLeftBack;
    private double distanceTravelled;
    private double desiredDistance;
    private double desiredAngle;
    private double desiredHeading;
    private double pathSpeed;
    private double targetX;
    private double targetY;

    private Timer timer;

    private IAutonomousLogger autonomousLogger;
    private String legName;


    public AutoDriveToCargoCommand(double distance, double strafeAngle, double speed, double heading, DriveSubsystem driveSubsystem, VisionSubSystem visionSubsystem) {
        this(distance, strafeAngle, speed, heading, driveSubsystem, visionSubsystem,null, null);
    }

    public AutoDriveToCargoCommand(double distance, double strafeAngle, double speed, double heading, DriveSubsystem driveSubsystem, VisionSubSystem visionSubsystem, String legName, IAutonomousLogger autonomousLogger) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem, visionSubsystem);

        desiredDistance = distance;
        desiredAngle = strafeAngle;
        desiredHeading = heading;
        pathSpeed = speed;

        this.legName = legName;
        this.autonomousLogger = autonomousLogger;

        this.timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        initialPositionRightFront = driveSubsystem.getDriveMotorPositionRightFront(); //looks at the encoder on one drive motor
        initialPositionLeftFront = driveSubsystem.getDriveMotorPositionLeftFront();
        initialPositionRightBack = driveSubsystem.getDriveMotorPositionRightBack();
        initialPositionLeftBack = driveSubsystem.getDriveMotorPositionLeftBack();

        //set targetX to -1 so we know if we didn't see a ball
        targetX = -1;

        if (autonomousLogger != null) {
            if (legName == null) {
                autonomousLogger.setLegName(getClass().getName());
            } else {
                autonomousLogger.setLegName(legName);
            }
            autonomousLogger.setInitialDrivePositions(initialPositionLeftFront, initialPositionRightFront, initialPositionLeftBack, initialPositionRightBack);
            autonomousLogger.setCurrentDrivePositions(initialPositionLeftFront, initialPositionRightFront, initialPositionLeftBack, initialPositionRightBack);
            autonomousLogger.setElapsed(0.0);
            autonomousLogger.doLog();
            timer.reset();
            timer.start();
        }
        driveSubsystem.setAutoSpinMode();
        driveSubsystem.setTargetHeading(desiredHeading);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double spinX = driveSubsystem.getSpinPower();

        // change heading if robot sees ball
        targetX = visionSubsystem.getBallXLocation();
        targetY = visionSubsystem.getBallYLocation();

        if(targetX < 0){
            //robot doesn't see ball, do original heading
            double desiredAngleRelativeToRobot = desiredAngle - driveSubsystem.getNavXFixedAngle();
            driveSubsystem.autoDrive(desiredAngleRelativeToRobot, pathSpeed, spinX);
        }
        else if(targetX > 1){
            //should never happen, default to original heading
            double desiredAngleRelativeToRobot = desiredAngle - driveSubsystem.getNavXFixedAngle();
            driveSubsystem.autoDrive(desiredAngleRelativeToRobot, pathSpeed, spinX);
        }
        else{
            //robot sees ball, adjust heading towards ball
            double desiredAngleRelativeToRobot  = ((targetX - 0.5)/0.0825)*5;
            driveSubsystem.autoDrive(desiredAngleRelativeToRobot, pathSpeed, spinX);
            driveSubsystem.setTargetHeading(driveSubsystem.getNavXFixedAngle() + desiredAngleRelativeToRobot);
        }

        // need to correct for what direction we are heading
        //double desiredAngleRelativeToRobot = desiredAngle - driveSubsystem.getNavXFixedAngle();
        //driveSubsystem.autoDrive(desiredAngleRelativeToRobot, pathSpeed, spinX);

        double currentPositionRightFront = driveSubsystem.getDriveMotorPositionRightFront();
        double currentPositionLeftFront = driveSubsystem.getDriveMotorPositionLeftFront();
        double currentPositionRightBack = driveSubsystem.getDriveMotorPositionRightBack();
        double currentPositionLeftBack = driveSubsystem.getDriveMotorPositionLeftBack();

        double distanceTravelledRightFront = Math.abs(currentPositionRightFront - initialPositionRightFront);
        double distanceTravelledLeftFront = Math.abs(currentPositionLeftFront - initialPositionLeftFront);
        double distanceTravelledRightBack = Math.abs(currentPositionRightBack - initialPositionRightBack);
        double distanceTravelledLeftBack = Math.abs(currentPositionLeftBack - initialPositionLeftBack);

        distanceTravelled = (distanceTravelledRightFront + distanceTravelledLeftFront + distanceTravelledRightBack + distanceTravelledLeftBack) / 4;
        
        if ((targetY >= 0.4) && (targetY <= 0.6)){
            desiredDistance = distanceTravelled + (34 + ((0.6 - targetY)*1.5));
        }

        if (autonomousLogger != null) {
          autonomousLogger.setCurrentDrivePositions(currentPositionLeftFront, currentPositionRightFront, currentPositionLeftBack, currentPositionRightBack);
          autonomousLogger.setElapsed(timer.get());
          autonomousLogger.doLog();
        }

        SmartDashboard.putNumber("AutoDrive.desiredDistance", desiredDistance);



    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.teleOpDrive(0, 0, 0);
        if (autonomousLogger != null) {
          timer.stop();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (distanceTravelled >= desiredDistance) {
            return true;
        }
        return false;
    }
}
