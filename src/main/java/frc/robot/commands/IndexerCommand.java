/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexerCommand extends CommandBase {
  private final IndexerSubsystem indexerSubsystem;
  private final Joystick gamepad;

  /**
   * Creates a new IndexerCommand.
   */
  public IndexerCommand(final IndexerSubsystem indexerSubsystem, final Joystick gamepad) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexerSubsystem = indexerSubsystem;
    addRequirements(indexerSubsystem);
    this.gamepad = gamepad;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = gamepad.getRawAxis(Constants.leftTrigger);
    boolean backward = gamepad.getPOV() == 180;
    boolean lock = gamepad.getRawButton(Constants.lockButton);
    indexerSubsystem.setConveyorSpeed(backward ? speed : -speed);
    indexerSubsystem.lock(lock);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
