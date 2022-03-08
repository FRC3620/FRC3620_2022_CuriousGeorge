/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import org.usfirst.frc3620.misc.*;

import frc.robot.commands.*;
import frc.robot.miscellaneous.CANSparkMaxSendable;
import frc.robot.subsystems.*;
import frc.robot.subsystems.RumbleSubsystem.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * 
 * @version 11 February 2020
 * 
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final static Logger logger = EventLogging.getLogger(RobotContainer.class, Level.INFO);
  final static int DRIVER_JOYSTICK_PORT = 0;
  final static int OPERATOR_JOYSTICK_PORT = 1;

  public final static double DRIVE_CLOSED_LOOP_RAMP_RATE_CONSTANT = 0.3;
  public final static double AZIMUTH_CLOSED_LOOP_RAMP_RATE_CONSTANT = 0.3;

  // need this
  public static CANDeviceFinder canDeviceFinder;
  public static RobotParameters2022 robotParameters;

  // hardware here...
  private static DigitalInput practiceBotJumper;

  // drive subsystem hardware
  public static CANSparkMaxSendable driveSubsystemRightFrontDrive;
  public static CANSparkMaxSendable driveSubsystemRightFrontAzimuth;
  public static RelativeEncoder driveSubsystemRightFrontDriveEncoder;
  public static RelativeEncoder driveSubsystemRightFrontAzimuthEncoder;
  public static AnalogInput driveSubsystemRightFrontHomeEncoder;
  
  public static CANSparkMaxSendable driveSubsystemLeftFrontDrive;
  public static CANSparkMaxSendable driveSubsystemLeftFrontAzimuth;
  public static RelativeEncoder driveSubsystemLeftFrontDriveEncoder;
  public static RelativeEncoder driveSubsystemLeftFrontAzimuthEncoder;
  public static AnalogInput driveSubsystemLeftFrontHomeEncoder;
  
  public static CANSparkMaxSendable driveSubsystemLeftBackDrive;
  public static CANSparkMaxSendable driveSubsystemLeftBackAzimuth;
  public static RelativeEncoder driveSubsystemLeftBackDriveEncoder;
  public static RelativeEncoder driveSubsystemLeftBackAzimuthEncoder;
  public static AnalogInput driveSubsystemLeftBackHomeEncoder;
  
  public static CANSparkMaxSendable driveSubsystemRightBackDrive;
  public static CANSparkMaxSendable driveSubsystemRightBackAzimuth;
  public static RelativeEncoder driveSubsystemRightBackDriveEncoder;
  public static RelativeEncoder driveSubsystemRightBackAzimuthEncoder;
  public static AnalogInput driveSubsystemRightBackHomeEncoder;

  //intake
  public static CANSparkMaxSendable intakeWheelbar;
  public static CANSparkMaxSendable intakeBelt;
  public static DoubleSolenoid intakeArm;

  // vision
  public static Solenoid ringLight;

  // shooter hardware variables are currently unknown so we need to change them
  public static WPI_TalonFX shooterSubsystemMainShooter1;
  public static WPI_TalonFX shooterSubsystemMainShooter2;
  public static WPI_TalonFX shooterSubsystemBackSpinShooter;
  public static CANSparkMaxSendable shooterSubsystemHoodMax;
  public static DigitalInput hoodLimitSwitch;
  public static CANSparkMaxSendable preShooterSubsystemPreShooter;

  // turret
  public static CANSparkMaxSendable turretSubsystemturretSpinner;
  public static RelativeEncoder turretSubsystemturretEncoder;

  // climber
  public static DigitalInput climberStationaryHookContact;
  public static CANSparkMaxSendable climberExtentionMotor; 
  public static DoubleSolenoid climberArmTilt;

  // subsystems here...
  public static DriveSubsystem driveSubsystem;
  public static VisionSubsystem visionSubsystem;
  public static ClimberSubsystem climberSubsystem; 
  public static IntakeSubsystem intakeSubsystem;
  public static IntakeArmSubsystem intakeArmSubsystem;
  public static TurretSubsystem turretSubsystem;
  public static ShooterSubsystem shooterSubsystem;
  public static ArmSubsystem armSubsystem;
  public static PreShooterSubsystem preShooterSubsystem;
  
  public static RumbleSubsystem operatorRumbleSubsystem;
  public static RumbleSubsystem driverRumbleSubsystem;

  // joysticks here....
  public static Joystick driverJoystick;
  public static Joystick operatorJoystick;

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    canDeviceFinder = new CANDeviceFinder();
    logger.info ("CAN bus: " + canDeviceFinder.getDeviceSet());

    robotParameters = (RobotParameters2022) RobotParametersContainer.getRobotParameters(RobotParameters2022.class);

    makeHardware();
    setupMotors();
    makeSubsystems();
    // Configure the button bindings
    configureButtonBindings();
    setupSmartDashboardCommands();
    setupAutonomousCommands();
  }

  void makeHardware() {
    practiceBotJumper = new DigitalInput(0);
    boolean shouldMakeAllCANDevices = shouldMakeAllCANDevices();
    if (!shouldMakeAllCANDevices) {
      logger.warn ("will try to deal with missing hardware!");
    }

    driveSubsystemRightFrontHomeEncoder = new AnalogInput(0);
    driveSubsystemLeftFrontHomeEncoder = new AnalogInput(1);
    driveSubsystemLeftBackHomeEncoder = new AnalogInput(2);
    driveSubsystemRightBackHomeEncoder = new AnalogInput(3);
      
    // we don't *need* to use the canDeviceFinder for CAN Talons because
    // they do not put up unreasonable amounts of SPAM
    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 1, "Swerve") || shouldMakeAllCANDevices){

      driveSubsystemRightFrontDrive = new CANSparkMaxSendable(1, MotorType.kBrushless);
      driveSubsystemRightFrontDriveEncoder = driveSubsystemRightFrontDrive.getEncoder();
      
      driveSubsystemRightFrontAzimuth = new CANSparkMaxSendable(2, MotorType.kBrushless);
      driveSubsystemRightFrontAzimuthEncoder = driveSubsystemRightFrontAzimuth.getEncoder();

      driveSubsystemLeftFrontDrive = new CANSparkMaxSendable(3, MotorType.kBrushless);
      driveSubsystemLeftFrontDriveEncoder = driveSubsystemLeftFrontDrive.getEncoder();
              
      driveSubsystemLeftFrontAzimuth = new CANSparkMaxSendable(4, MotorType.kBrushless);
      driveSubsystemLeftFrontAzimuthEncoder = driveSubsystemLeftFrontAzimuth.getEncoder();

      driveSubsystemLeftBackDrive = new CANSparkMaxSendable(5, MotorType.kBrushless);
      driveSubsystemLeftBackDriveEncoder = driveSubsystemLeftBackDrive.getEncoder();
              
      driveSubsystemLeftBackAzimuth = new CANSparkMaxSendable(6, MotorType.kBrushless);
      driveSubsystemLeftBackAzimuthEncoder = driveSubsystemLeftBackAzimuth.getEncoder();

      driveSubsystemRightBackDrive = new CANSparkMaxSendable(7, MotorType.kBrushless);
      driveSubsystemRightBackDriveEncoder = driveSubsystemRightBackDrive.getEncoder();
      
      driveSubsystemRightBackAzimuth = new CANSparkMaxSendable(8, MotorType.kBrushless);
      driveSubsystemRightBackAzimuthEncoder = driveSubsystemRightBackAzimuth.getEncoder();
    }

    climberStationaryHookContact = new DigitalInput(1);
    if (robotParameters.hasClimber()) {
      if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 17, "climberExtentionMotor") || shouldMakeAllCANDevices) {
        climberExtentionMotor = new CANSparkMaxSendable(17, MotorType.kBrushless);
      }
    } else {
      logger.info ("robot parameters say no climber, so skipping");
    }

    // shooter motors
    if (robotParameters.hasShooter()) {
      if (canDeviceFinder.isDevicePresent(CANDeviceType.TALON, 13, "top shooter 1") || shouldMakeAllCANDevices) {
        // Shooter Motors
        shooterSubsystemMainShooter1 = new WPI_TalonFX(13);
      }

      if (canDeviceFinder.isDevicePresent(CANDeviceType.TALON, 14, "top shooter 2") || shouldMakeAllCANDevices) {
        shooterSubsystemMainShooter2 = new WPI_TalonFX(14);
      }

      if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 11, "preshooter") || shouldMakeAllCANDevices) {
        preShooterSubsystemPreShooter = new CANSparkMaxSendable(11, MotorType.kBrushless);
      }

      if (canDeviceFinder.isDevicePresent(CANDeviceType.TALON, 15, "back shooter") || shouldMakeAllCANDevices) {
        shooterSubsystemBackSpinShooter = new WPI_TalonFX(15);
      }

      if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 16)){
        shooterSubsystemHoodMax = new CANSparkMaxSendable(16, MotorType.kBrushless);
      }
    } else {
      logger.info ("robot parameters say no shooter, so skipping");
    }

    // turret
    if (robotParameters.hasTurret()){
      if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 12, "turret") || shouldMakeAllCANDevices) {
        turretSubsystemturretSpinner = new CANSparkMaxSendable(12, MotorType.kBrushless);
        turretSubsystemturretEncoder = turretSubsystemturretSpinner.getEncoder();
      }
    } else {
      logger.info ("robot parameters say no turret, so skipping");
    }

    //intake
    if (robotParameters.hasIntake()) {
      if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 9, "wheel bar") || shouldMakeAllCANDevices) {
        intakeWheelbar = new CANSparkMaxSendable(9, MotorType.kBrushless);
      }
      
      if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 10, "Intake Belt") || shouldMakeAllCANDevices) {
        intakeBelt = new CANSparkMaxSendable(10, MotorType.kBrushless);
      }
        
    } else {
      logger.info ("robot parameters say no intake, so skipping");
    }

    PneumaticsModuleType pneumaticModuleType = null;

    if (canDeviceFinder.isDevicePresent(CANDeviceType.REV_PH, 1, "REV PH") || shouldMakeAllCANDevices) {
      pneumaticModuleType = PneumaticsModuleType.REVPH;
    } else if (canDeviceFinder.isDevicePresent(CANDeviceType.CTRE_PCM, 0, "CTRE PCM")) {
      pneumaticModuleType = PneumaticsModuleType.CTREPCM;
    }

    if (pneumaticModuleType != null) {
      Compressor compressor = new Compressor(pneumaticModuleType);
      if (! robotParameters.shouldRunCompressor()) {
        logger.info ("disabling the compressor because of robot_parameters");
        compressor.disable();
      } else {
        compressor.enableDigital();
      }

      ringLight = new Solenoid(pneumaticModuleType, 7);
      ringLight.set(true);
      if (robotParameters.hasClimber()){
        climberArmTilt = new DoubleSolenoid(pneumaticModuleType, 0, 1);
      }
      if (robotParameters.hasIntake()){
        intakeArm = new DoubleSolenoid(pneumaticModuleType, 2, 3);
      }
    }
  }
  
  void setupMotors() {
    int kTimeoutMs = 0;

    if (driveSubsystemRightFrontDrive != null){

      resetMaxToKnownState(driveSubsystemRightFrontDrive, true);
      driveSubsystemRightFrontDrive.setClosedLoopRampRate(DRIVE_CLOSED_LOOP_RAMP_RATE_CONSTANT);

      resetMaxToKnownState(driveSubsystemRightFrontAzimuth, false);
      driveSubsystemRightFrontAzimuth.setClosedLoopRampRate(AZIMUTH_CLOSED_LOOP_RAMP_RATE_CONSTANT);

      resetMaxToKnownState(driveSubsystemLeftFrontDrive, true);
      driveSubsystemLeftFrontDrive.setClosedLoopRampRate(DRIVE_CLOSED_LOOP_RAMP_RATE_CONSTANT);

      resetMaxToKnownState(driveSubsystemLeftFrontAzimuth, false);
      driveSubsystemLeftFrontAzimuth.setClosedLoopRampRate(AZIMUTH_CLOSED_LOOP_RAMP_RATE_CONSTANT);

      resetMaxToKnownState(driveSubsystemLeftBackDrive, true);
      driveSubsystemLeftBackDrive.setClosedLoopRampRate(DRIVE_CLOSED_LOOP_RAMP_RATE_CONSTANT);

      resetMaxToKnownState(driveSubsystemLeftBackAzimuth, false);
      driveSubsystemLeftBackAzimuth.setClosedLoopRampRate(AZIMUTH_CLOSED_LOOP_RAMP_RATE_CONSTANT);

      resetMaxToKnownState(driveSubsystemRightBackDrive, true);
      driveSubsystemRightBackDrive.setClosedLoopRampRate(DRIVE_CLOSED_LOOP_RAMP_RATE_CONSTANT);
      
      resetMaxToKnownState(driveSubsystemRightBackAzimuth, false);
      driveSubsystemRightBackAzimuth.setClosedLoopRampRate(AZIMUTH_CLOSED_LOOP_RAMP_RATE_CONSTANT);
    }
    if (shooterSubsystemMainShooter1 != null) {
      resetTalonFXToKnownState(shooterSubsystemMainShooter1, InvertType.InvertMotorOutput);
    }

    if (shooterSubsystemMainShooter2 != null) {
      resetTalonFXToKnownState(shooterSubsystemMainShooter2, InvertType.OpposeMaster);
      shooterSubsystemMainShooter2.follow(shooterSubsystemMainShooter1);
    }

    if (shooterSubsystemBackSpinShooter != null) {
      resetTalonFXToKnownState(shooterSubsystemBackSpinShooter, InvertType.InvertMotorOutput);
    }

    if(preShooterSubsystemPreShooter != null) {
      resetMaxToKnownState(preShooterSubsystemPreShooter, true); 
    }

    if(shooterSubsystemHoodMax != null) {
      resetMaxToKnownState(shooterSubsystemHoodMax, true);
      shooterSubsystemHoodMax.setIdleMode(IdleMode.kCoast);
      shooterSubsystemHoodMax.setOpenLoopRampRate(.3);
      shooterSubsystemHoodMax.setClosedLoopRampRate(.3);
      shooterSubsystemHoodMax.setSmartCurrentLimit(10);
  }

    if(climberExtentionMotor != null) {
      resetMaxToKnownState(climberExtentionMotor, true);
      climberExtentionMotor.setIdleMode(IdleMode.kBrake);
    }

    if(intakeWheelbar != null) {
      resetMaxToKnownState(intakeWheelbar, false);
      intakeWheelbar.setSmartCurrentLimit(20);
    }

    if(intakeBelt != null) {
      resetMaxToKnownState(intakeBelt, false);
      intakeBelt.setSmartCurrentLimit(20);
    }

    if(turretSubsystemturretSpinner != null) {
      resetMaxToKnownState(turretSubsystemturretSpinner, true);
      turretSubsystemturretSpinner.setSmartCurrentLimit(10);
    }
  }

  static void resetMaxToKnownState(CANSparkMaxSendable x, boolean inverted) {
    // TODO set to factory default 
    x.setInverted(inverted);
    x.setIdleMode(IdleMode.kCoast);
    x.setOpenLoopRampRate(1);
    x.setClosedLoopRampRate(1);
    x.setSmartCurrentLimit(80);
  }

  static void resetTalonFXToKnownState(WPI_TalonFX m, InvertType invert) {
    int kTimeoutMs = 0;
    m.configFactoryDefault();
    m.setInverted(invert);

    /*

    //set max and minimum(nominal) speed in percentage output
    m.configNominalOutputForward(+1, kTimeoutMs);
    m.configNominalOutputReverse(-1, kTimeoutMs);
    m.configPeakOutputForward(+1, kTimeoutMs);
    m.configPeakOutputReverse(-1, kTimeoutMs);
    
    StatorCurrentLimitConfiguration amprage=new StatorCurrentLimitConfiguration(true,40,0,0); 
    m.configStatorCurrentLimit(amprage);
    */
    m.setNeutralMode(NeutralMode.Coast);
  }

  void makeSubsystems() {
    driveSubsystem = new DriveSubsystem();
    visionSubsystem = new VisionSubsystem();
    climberSubsystem = new ClimberSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    intakeArmSubsystem = new IntakeArmSubsystem();
    turretSubsystem = new TurretSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    armSubsystem = new ArmSubsystem();
    preShooterSubsystem = new PreShooterSubsystem();
    operatorRumbleSubsystem = new RumbleSubsystem(1);
    driverRumbleSubsystem = new RumbleSubsystem(0);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link edu.wpi.first.wpilibj.XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverJoystick = new Joystick(DRIVER_JOYSTICK_PORT);
    operatorJoystick = new Joystick(OPERATOR_JOYSTICK_PORT);

    //Dpad 
    DPad driverDPad = new DPad(driverJoystick, 0);
    DPad operatorDPad = new DPad(operatorJoystick, 0);
   
    //Dead reckon shots
    //Sets up turret and shooter and hood
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_A).whenPressed(new GetVisionReadyToShootCommand());
    //B for bloop shot up against lower hub
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_B).whenPressed(new GetBloopReadyToShootCommand());
    //X is the dead reckon from tarmac shot
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_X).whenPressed(new GetTarmacReadyToShootCommand());
    //Y is the dead reckon from the safe zone
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_Y).whenPressed(new GetSafeZoneReadyToShootCommnd());

    //Climber buttons
    //left interior
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_BACK).whenPressed(new ClimberTiltTestCommandOut());
    //right interior
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_START).whenPressed(new ClimberTiltTestCommandIn());

    //Shooter
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_LEFT_STICK).whenPressed(new ShooterOffCommand());

    operatorDPad.up().whenPressed(new MoveTurretCommand(turretSubsystem, -5));
    operatorDPad.right().whenPressed(new MoveTurretCommand(turretSubsystem, 90));
    operatorDPad.down().whenPressed(new MoveTurretCommand(turretSubsystem, 175));
    operatorDPad.left().whenPressed(new MoveTurretCommand(turretSubsystem, 270));
   

    JoystickButton centerOnBallButton = new JoystickButton(driverJoystick, XBoxConstants.BUTTON_Y);
    centerOnBallButton.whileHeld(new InstantCenterOnBallCommand(driveSubsystem, visionSubsystem));
    JoystickButton resetNavXButton = new JoystickButton(driverJoystick, XBoxConstants.BUTTON_X);
    resetNavXButton.whenPressed(new ResetNavXCommand(driveSubsystem));
    JoystickButton stopDriveButton = new JoystickButton(driverJoystick, XBoxConstants.BUTTON_RIGHT_BUMPER);
    stopDriveButton.toggleWhenPressed(new StopDriveCommand(driveSubsystem));

    JoystickButton intakeButton = new JoystickButton(driverJoystick, XBoxConstants.BUTTON_LEFT_BUMPER);
    intakeButton.toggleWhenPressed(new IntakeOnCommand());
    JoystickButton intakeArmButton = new JoystickButton(driverJoystick, XBoxConstants.BUTTON_B);
    intakeArmButton.toggleWhenPressed(new IntakeArmCommand());

    // driver right trigger fires
    new TriggerButton(driverJoystick, false).whenPressed(new ShootCommand());

    JoystickButton ejectButton = new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_RIGHT_BUMPER);
    ejectButton.whileHeld(new EjectBallCommand());
  }

  public static double getOperatorJoystickRightY() {
    double axisValue = operatorJoystick.getRawAxis(XBoxConstants.AXIS_RIGHT_Y); //Grabs the joystick value
    if (axisValue < 0.2 && axisValue > -0.2) { //Since the joystick doesnt stay at zero, make it not give a false value
      return 0;
    }
    return -axisValue;
  }

  void setupSmartDashboardCommands() {
    SmartDashboard.putData(new ZeroDriveEncodersCommand(driveSubsystem));
  
    SmartDashboard.putData("TestAuto", new TestAuto(driveSubsystem));
    SmartDashboard.putData("AutoDriveToCargo Test", new DriveToCargoTestAuto(driveSubsystem, visionSubsystem));
    SmartDashboard.putData("5 Ball Auto P", new FiveBallAuto(driveSubsystem, visionSubsystem));
    SmartDashboard.putData("4 Ball Auto P", new FourBallAutoP(driveSubsystem, visionSubsystem));
    SmartDashboard.putData("4 Ball Auto Q", new FourBallAutoQ(driveSubsystem, visionSubsystem));
    SmartDashboard.putData("3 Ball Auto Q", new ThreeBallAutoQ(driveSubsystem,  visionSubsystem));

    SmartDashboard.putData("Reset NavX", new ResetNavXCommand(driveSubsystem));
    SmartDashboard.putData("Toggle field relative", new ToggleFieldRelativeModeCommand(driveSubsystem));
    SmartDashboard.putData("Find target",new FindTargetCommand(turretSubsystem, visionSubsystem));


    SmartDashboard.putData("Shooter Test Command", new ShooterTestCommand(shooterSubsystem));

    SmartDashboard.putData("Eject Ball", new EjectBallCommand());
    SmartDashboard.putData("Intake Ball", new IntakeOnCommand());
    SmartDashboard.putData("preshooter with intake", new ShootCommand());

    SmartDashboard.putData("Climber Tilt Out", new ClimberTiltTestCommandOut());
    SmartDashboard.putData("Climber Tilt In", new ClimberTiltTestCommandIn());

    SmartDashboard.putData("Find Target",new FindTargetCommand(turretSubsystem, visionSubsystem));
    SmartDashboard.putData("Find Target and Wiggle", new MassageOperatorToShootCommand(visionSubsystem, driverRumbleSubsystem));

    SmartDashboard.putData("Rumble Command", new RumbleCommand(driverRumbleSubsystem, Hand.BOTH, 1.0, 3.0));
    SmartDashboard.putData("Rumble Command 2", new RumbleCommand(driverRumbleSubsystem, .5, 5.0));
    SmartDashboard.putData("Rumble Command 3", new RumbleCommand(driverRumbleSubsystem,Hand.LEFT, .5, 3.0));
    SmartDashboard.putData("Rumble Command 4", new RumbleCommand(driverRumbleSubsystem,Hand.RIGHT, .5, 3.0));
    SmartDashboard.putData("Top shooter to 0.1", new ShooterPowerTest());

    SmartDashboard.putData("Get ready to climb", new GetReadyToClimbCommand());

    SmartDashboard.putData("Shoot", new ShootCommand());
  }

  SendableChooser<Command> chooser = new SendableChooser<>();
  public void setupAutonomousCommands() {
    SmartDashboard.putData("Auto mode", chooser);

    chooser.addOption("TestAuto", new TestAuto(driveSubsystem));
    chooser.addOption("AutoDriveToCargo Test", new DriveToCargoTestAuto(driveSubsystem, visionSubsystem));
    chooser.addOption("5 Ball P Auto", new FiveBallAuto(driveSubsystem, visionSubsystem));
    chooser.addOption("4 Ball P Auto", new FourBallAutoP(driveSubsystem, visionSubsystem));
    chooser.addOption("4 Ball Q Auto", new FourBallAutoQ(driveSubsystem, visionSubsystem));
    chooser.addOption("3 Ball Q Auto", new ThreeBallAutoQ(driveSubsystem, visionSubsystem));
  }
  
  static double driverStrafeDeadzone = 0.1;
  public static double getDriveVerticalJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_Y);
    SmartDashboard.putNumber("driver.raw.y", axisValue);
    if (Math.abs(axisValue) < driverStrafeDeadzone) {
      return 0;
    }
    if (axisValue < 0){
      return (axisValue*axisValue);
    }
    return -axisValue*axisValue;
  }

  public static double getDriveHorizontalJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_X);
    SmartDashboard.putNumber("driver.raw.x", axisValue);
    if (Math.abs(axisValue) < driverStrafeDeadzone) {
      return 0;
    }
    if (axisValue < 0){
      return -(axisValue*axisValue);
    }
    return axisValue*axisValue;
  }

  static double driverSpinDeadzone = 0.1;
  public static double getDriveSpinJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_RIGHT_X);
    SmartDashboard.putNumber("driver.raw.spin", axisValue);
    if (Math.abs(axisValue) < driverSpinDeadzone) {
      return 0;
    }
    if (axisValue < 0){
      return -(axisValue*axisValue);
    }
    return axisValue*axisValue;
  }

  static double operatorDeadzone = 0.1;
    
  /**
   * return the position of the operator's right vertical joystick
   * @return the joystick position, positive is up (away from driver), negative is down
   * pulled towards driver.
   */
  public static double getOperatorVerticalJoystick() {
    double axisValue = operatorJoystick.getRawAxis(XBoxConstants.AXIS_RIGHT_Y);
    if (Math.abs(axisValue) < operatorDeadzone) {
      return 0;
    }
    return -axisValue;
  }

  /**
   * Determine if we should make software objects, even if the device does 
   * not appear on the CAN bus.
   *
   * We should if it's connected to an FMS.
   *
   * We should if it is missing a grounding jumper on DigitalInput 0.
   *
   * We should if the robot_parameters.json says so for this MAC address.
   * 
   * @return true if we should make all software objects for CAN devices
   */
  public static boolean shouldMakeAllCANDevices() {
    if (DriverStation.isFMSAttached()) {
      return true;
    }

    if(practiceBotJumper.get() == true){
      return true;
    }

    if (robotParameters.shouldMakeAllCANDevices()) {
      return true;
    }

    return false;
  }

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new GoldenAutoCommand(driveSubsystem, shooterSubsystem, VisionSubsystem, intakeSubsystem);
    return chooser.getSelected();
  }
}