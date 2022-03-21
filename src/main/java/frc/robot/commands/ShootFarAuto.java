package frc.robot.commands;
import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShootFarAuto extends SequentialCommandGroup {
    Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

    public ShootFarAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem, IntakeSubsystem intakeSubsystem){
        addCommands(
            new StartShooterDataLoggingCommand(getClass().getSimpleName(), 20.0),
            new MoveTurretCommand(turretSubsystem, 180), 
            new SequentialCommandGroup(
                new AutoShootCommand(),
                new PullTheTriggerForOneCommand(),
                new AutoPushBallUpCommand(),
                new PullTheTriggerForOneCommand(),
                new AutoPushBallUpCommand(),
                new PullTheTriggerForOneCommand()
            )
        );
    }
}