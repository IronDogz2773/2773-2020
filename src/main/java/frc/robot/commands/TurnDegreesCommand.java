/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import edu.wpi.first.wpilibj.controller.PIDController;

public class TurnDegreesCommand extends CommandBase {
  PIDController pidController = new PIDController(.025, 0.03, 0.0);
  private final DriveSubsystem driveSubsystem;
  private double rotation;
  private final double angle;
  private double target;
  private final NavigationSubsystem nav;

  /**
   * Creates a new TurnDegreesCommand.
   */
  public TurnDegreesCommand(final DriveSubsystem subsystem, final NavigationSubsystem nav, final double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSubsystem = subsystem;
    addRequirements(subsystem);
    this.angle = angle;
    this.nav = nav;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = nav.getGyroAngle() + angle;
    pidController.setSetpoint(target);
    pidController.setTolerance(2.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!pidController.atSetpoint()) {
      rotation = pidController.calculate(nav.getGyroAngle());
      rotation = MathUtil.clamp(rotation, -.8, .8);
      driveSubsystem.rawDrive(0, rotation);
    }
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
