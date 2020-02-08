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
import edu.wpi.first.wpilibj.controller.PIDController;

public class TurnDegreesCommand extends CommandBase {
  PIDController pidController = new PIDController(.025, 0.03, 0.0);
    private final DriveSubsystem driveSubsystem;
    private double rot;
    private double angle;
    private double target;
  /**
   * Creates a new TurnDegreesCommand.
   */
  public TurnDegreesCommand(DriveSubsystem subsystem, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    driveSubsystem = subsystem;
    addRequirements(subsystem);
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = driveSubsystem.gyroscope.getAngle() + angle;
    pidController.setSetpoint(target);
    pidController.setTolerance(2.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!pidController.atSetpoint())
    {
      rot = pidController.calculate(driveSubsystem.gyroscope.getAngle());
      rot = MathUtil.clamp(rot, -.8, .8);
      driveSubsystem.rawDrive(0, rot);
    }
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
