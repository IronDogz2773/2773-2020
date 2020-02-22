/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

public class DriveDistanceCommand extends CommandBase {
  PIDController pidController;
  private final DriveSubsystem driveSubsystem;
  private final NavigationSubsystem navigationSubsystem;
  private double distance;
  private double target;
  private double speed;
  /**
   * Creates a new DriveDistanceCommand.
   */
  public DriveDistanceCommand(final DriveSubsystem driveSubsystem, final NavigationSubsystem navigationSubsystem, final double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.navigationSubsystem = navigationSubsystem;
    this.distance = distance;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController = new PIDController(0, 0, 0);
    target = distance;
    pidController.setSetpoint(target);
    pidController.setTolerance(0);
    /* 
    ---UNCOMMENT FOR TESTING---
    final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    final NetworkTable pidTable = inst.getTable("PID");
    double Kp = pidTable.getEntry("P").getDouble(0);
    double Ki = pidTable.getEntry("I").getDouble(0);
    double Kd = pidTable.getEntry("D").getDouble(0);
    pidController = new PIDController(Kp, Ki, Kd);*/
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speed = pidController.calculate();
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
