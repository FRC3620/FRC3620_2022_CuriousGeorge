package frc.robot.commands;
import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
public class ShootFarAutoCommand extends SequentialCommandGroup {
    Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

    public ShootFarAutoCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, TurretSubsystem turretSubsystem, IntakeSubsystem intakeSubsystem){
        addCommands(
            new StartShooterDataLoggingCommand(getClass().getSimpleName(), 20.0),
            new MoveTurretCommand(turretSubsystem, 180), 
            new LogCommand("done with 180"),
            new AutoShootCommand(),
            new LogCommand("found target"),
            new ParallelCommandGroup(
                new IntakeOnCommand(),
                new SequentialCommandGroup(
                    new PullTheTriggerForOneCommand(),
                    new LogCommand("shot once"),
                    new AutoPushBallUpCommand(),
                    new LogCommand("ball is ready to shoot"),
                    new PullTheTriggerForOneCommand(),
                    new AutoPushBallUpCommand(),
                    new PullTheTriggerCommand()
                )
            )
        );
 
    }
    class LogCommand extends InstantCommand {
        String m;
        Object o;
        LogCommand(String message) {
          this.m = message;
          this.o = null;
        }
    
        LogCommand(String message, Object[] args) {
          this.m = message;
          this.o = args;
        }
    
        @Override
        public void initialize() {
          if (o == null) {
            logger.info(m);
          } else {
            logger.info(m, o);
          }
        }
      
        @Override
        public boolean runsWhenDisabled() {
          return true;
        }
      }
    }
    
