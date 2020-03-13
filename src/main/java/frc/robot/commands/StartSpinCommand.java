/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;

public class StartSpinCommand extends CommandBase {
  private final ShooterSubsystem shooter;
  private final Joystick gamepad;
  private boolean running;

  /**
   * Creates a new StartSpinCommand.
   */
  public StartSpinCommand(final ShooterSubsystem shooter, final Joystick gamepad) {
    this.shooter = shooter;
    this.gamepad = gamepad;
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;
    if(Math.abs(gamepad.getRawAxis(Constants.rightJoystickY)) > .2)
    {
      speed = gamepad.getRawAxis(Constants.rightJoystickY);
      running = true;
    }
    else
    {
      speed = 0;
      running = false;
    }
    shooter.startSpin(speed);
    SmartDashboard.putBoolean("Shooter Running", running);
    SmartDashboard.putNumber("Shooter RPM, %",shooter.checkSpinRate() * 100);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    running = false;
    shooter.stopSpin();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
