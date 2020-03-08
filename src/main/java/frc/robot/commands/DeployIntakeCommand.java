/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DeployIntakeCommand extends CommandBase {
  private static final double FORWARD_DURATION = 0.1;
  private static final double COMMAND_DURATION = FORWARD_DURATION * 2;
  private final DriveSubsystem driveSubsystem;
  private Timer timer;
  private boolean movingBack;

  /**
   * Creates a new DeployIntakeCommand.
   */
  public DeployIntakeCommand(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);

    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    driveSubsystem.rawDrive(1, 0);
    movingBack = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() >= FORWARD_DURATION && !movingBack) {
      driveSubsystem.rawDrive(-1, 0);
      movingBack = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    driveSubsystem.rawDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= COMMAND_DURATION;
  }
}
