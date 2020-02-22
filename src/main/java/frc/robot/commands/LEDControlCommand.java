/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class LEDControlCommand extends CommandBase {

  private final LEDSubsystem ledSubystem;
  /**
   * Creates a new LEDControlCommand.
   */
  public LEDControlCommand(LEDSubsystem ledSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ledSubystem = ledSubsystem;
    addRequirements(ledSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(ledSubystem.getTeam().equals("red"))
    {
      ledSubystem.setLED(-0.29);
    }
    else if(ledSubystem.getTeam().equals("blue"))
    {
      ledSubystem.setLED(-0.31);
    }
    else 
    {
      ledSubystem.setLED(-0.13);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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
