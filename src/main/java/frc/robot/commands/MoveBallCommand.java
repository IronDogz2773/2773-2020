/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;

public class MoveBallCommand extends CommandBase {
  private static final double INDEXER_TIME = 3.0;
  private static final double LOCK_DURATION = 2.0;
  private final IndexerSubsystem indexerSubsystem;
  private Timer timer;

  /**
   * Creates a new MoveBallCommand.
   */
  public MoveBallCommand(IndexerSubsystem indexerSubsystem) {
    this.indexerSubsystem = indexerSubsystem;
    addRequirements(indexerSubsystem);

    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    indexerSubsystem.lock(false);
    indexerSubsystem.setConveyorSpeed(-Constants.indexerSpeedCap);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() >= LOCK_DURATION) {
      indexerSubsystem.lock(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.stopConveyor();
    indexerSubsystem.lock(false);
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() >= INDEXER_TIME;
  }
}
