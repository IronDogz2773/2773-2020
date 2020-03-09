/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbControllerCommand extends CommandBase {
  private final ClimbSubsystem climbSubsystem;
  private double rightSpeed;
  private double leftSpeed;
  private Joystick gamepad;
  
  /**
   * Creates a new ClimbControllerCommand.
   */
  public ClimbControllerCommand(ClimbSubsystem climbSubsystem, Joystick gamepad) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climbSubsystem = climbSubsystem;
    this.gamepad = gamepad;
    addRequirements(climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rightSpeed = gamepad.getRawAxis(5);
    leftSpeed = gamepad.getRawAxis(1);
    climbSubsystem.climb(rightSpeed, leftSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
