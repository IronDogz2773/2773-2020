/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.AutonomousBuilder;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

public class AutonomousCommand extends CommandBase {
  String trajectoryJSON;
  Trajectory trajectory;
  DifferentialDriveVoltageConstraint autoVoltageConstraint;
  DriveSubsystem driveSubsystem;
  NavigationSubsystem navigationSubsystem;
  AutonomousBuilder autonomousBuilder;

  /**
   * Creates a new getAutonomousCommand.
   */
  public AutonomousCommand(DriveSubsystem driveSubsystem, NavigationSubsystem navigationSubsystem, AutonomousBuilder autonomousBuilder) {
    this.driveSubsystem = driveSubsystem;
    this.navigationSubsystem = navigationSubsystem;
    this.autonomousBuilder = autonomousBuilder;
    addRequirements(driveSubsystem);
    
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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
