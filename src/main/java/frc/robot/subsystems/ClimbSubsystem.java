/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Represents a climber mechanism and its related components
 * @author Tyler Graham
 * @author irondogz@gmail.com
 */
public class ClimbSubsystem extends SubsystemBase {
  private final Spark climbRight = new Spark(Constants.rightClimbPort);
  private final Spark climbLeft = new Spark(Constants.leftClimbPort);

  /**
   * Creates a new ClimbSubsystem
   */
  public ClimbSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets right and left motor speeds accordingly
   * @param rightSpeed The speed value of the right motor between -1.0 and 1.0 to set
   * @param leftSpeed The speed value of the left motor between -1.0 and 1.0 to set
   */
  public void climb(double rightSpeed, double leftSpeed)
  {
    climbRight.set(rightSpeed);
    climbLeft.set(leftSpeed);
  }
}
