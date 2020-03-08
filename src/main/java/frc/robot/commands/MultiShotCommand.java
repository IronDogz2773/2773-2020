/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class MultiShotCommand extends CommandBase {
  private final ShooterSubsystem shooterSubsystem;
  private double numBalls;
  private int shooterState;
  private MoveBallCommand moveBallCommand;
  private final static int START_SPIN = 0;
  private final static int WAIT_FOR_SPIN = 1;
  private final static int MOVE_BALL = 2;
  private final static int STOP_SPIN = 3;

  /**
   * Creates a new MultiShotCommand.
   */
  public MultiShotCommand(final ShooterSubsystem shooterSubsystem, final IndexerSubsystem indexerSubsystem,
      final double balls) {
    this.shooterSubsystem = shooterSubsystem;
    this.moveBallCommand = new MoveBallCommand(indexerSubsystem);
    numBalls = balls;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterState = START_SPIN;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (shooterState) {
    case START_SPIN:
      shooterSubsystem.startSpin(1.0);
      shooterState = WAIT_FOR_SPIN;
      break;
    case WAIT_FOR_SPIN:
      // if shooter at rate, decreases number of balls and starts conveyor
      if (shooterSubsystem.atRate()) {
        numBalls--;
        // begin to move ball (using subcommand)
        moveBallCommand.initialize();
        shooterState = MOVE_BALL;
      }
      break;
    case MOVE_BALL:
      // invoke subcommand and check if it's finished
      moveBallCommand.execute();
      if (moveBallCommand.isFinished()) {
        moveBallCommand.end(false);
        // keeps repeating until no more balls
        if (numBalls > 0) {
          shooterState = WAIT_FOR_SPIN;
        } else {
          shooterSubsystem.stopSpin();
          shooterState = STOP_SPIN;
        }
      }
      break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    shooterSubsystem.stopSpin();
    if (interrupted && shooterState == MOVE_BALL) {
      // if interrupted, interupt running subcommand
      moveBallCommand.end(true);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterState == STOP_SPIN;
  }
}
