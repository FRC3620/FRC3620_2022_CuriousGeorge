package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeArmSubsystem;

public class IntakeArmCommand extends CommandBase {
  IntakeArmSubsystem intakeArmSubsystem = RobotContainer.intakeArmSubsystem;
  /** Creates a new IntakeBallCommand. */
  public IntakeArmCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intakeArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeArmSubsystem.extendIntakeArm();
  }

  @Override
  /**
   * Called every time the scheduler runs while the command is scheduled.
   */
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeArmSubsystem.retractIntakeArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
