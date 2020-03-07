/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SingleShotCommand extends CommandBase {
  private final ShooterSubsystem shooterSubsystem;
  private final IndexerSubsystem indexerSubsystem;
  private final double INDEXER_TIME = 1.0;
  private double numBalls;
  private final Timer timer;

  /**
   * Creates a new SingleShotCommand.
   */
  public SingleShotCommand(ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsystem, double balls) {
    this.shooterSubsystem = shooterSubsystem;
    this.indexerSubsystem = indexerSubsystem;
    timer = new Timer();
    numBalls = balls;
    addRequirements(shooterSubsystem, indexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.startSpin(1.0);
    if(shooterSubsystem.atRate())
    {
      timer.reset();
      if(timer.get() <= INDEXER_TIME && numBalls > 0)
        indexerSubsystem.startConveyorSpin(1.0);
      else
        numBalls--;
        //TODO add limit switch functionality hopefully
    }
    else
    {
      indexerSubsystem.stopConveyorSpin();
      timer.reset();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    shooterSubsystem.stopSpin();
    indexerSubsystem.stopConveyorSpin();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(numBalls == 0)
    {
      return true;
    }
    return false;
  }
}
