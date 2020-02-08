/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AirSubsystem;

public class CompressorControlCommand extends CommandBase {
  private final AirSubsystem airSubsystem;
  /**
   * Creates a new CompressorControlCommand.
   */
  public CompressorControlCommand(AirSubsystem airSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.airSubsystem = airSubsystem;
    addRequirements(airSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    airSubsystem.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    airSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
