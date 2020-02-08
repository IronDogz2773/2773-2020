/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSpinCommand extends CommandBase {
  private IntakeSubsystem intakeSubsystem;
  private Joystick joystick;
  /**
   * Creates a new IntakeSpinCommand.
   */
  public IntakeSpinCommand(IntakeSubsystem intakeSubsystem, Joystick joystick) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
    this.joystick = joystick;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(joystick.getRawAxis(2) > .2) //raw axis 2 is left trigger
      intakeSubsystem.startSpin(1);
    else if(joystick.getRawAxis(3) > .2) //raw axis 3 is right trigger
      intakeSubsystem.startSpin(-1);
    else
      intakeSubsystem.stopSpin();
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
