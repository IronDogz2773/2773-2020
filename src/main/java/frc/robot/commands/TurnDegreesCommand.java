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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurnDegreesCommand extends CommandBase {
  //Ku .05
  PIDController pidController;
  private final DriveSubsystem driveSubsystem;
  private double rotation;
  private double angle;
  private double target;
  private final NavigationSubsystem nav;

  /**
   * Creates a new TurnDegreesCommand.
   */
  public TurnDegreesCommand(final DriveSubsystem driveSubsystem, final NavigationSubsystem nav, final double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    this.angle = angle;
    this.nav = nav;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    final NetworkTable pidTable = inst.getTable("PID");
    double Kp = pidTable.getEntry("P").getDouble(0);
    double Ki = pidTable.getEntry("I").getDouble(0);
    double Kd = pidTable.getEntry("D").getDouble(0);
    pidController = new PIDController(Kp, Ki, Kd);
    angle = pidTable.getEntry("Test angle").getDouble(angle);
    driveSubsystem.driveState = true;
    target = nav.getGyroAngle() + angle;
    pidController.setTolerance(0);
    pidController.setSetpoint(target);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("rotation", rotation);
    SmartDashboard.putNumber("angle", angle);
    SmartDashboard.putNumber("target", target);
    if (!pidController.atSetpoint()) {
      rotation = pidController.calculate(nav.getGyroAngle());
      rotation = MathUtil.clamp(rotation, -.5, .5);
      driveSubsystem.rawDrive(0, rotation, false);
      
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    driveSubsystem.driveState = false;
    return false;
  }
}
