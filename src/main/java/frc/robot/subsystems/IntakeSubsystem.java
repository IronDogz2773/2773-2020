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
 * Represents an intake mechanism and its related components
 * @author Ilya Dzialendzik
 * @author Yury Dzialendzik
 * @author irondogzrobotics@gmail.com
 */
public class IntakeSubsystem extends SubsystemBase {
  private final Spark intake = new Spark(Constants.intakePort);

  /**
   * Creates a new IntakeSubsystem.
   */
  public IntakeSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Spins the intake mechanism motor at a specified speed
   * @param speed The speed value of the conveyor motors between -1.0 and 1.0 to set
   */
  public void startSpin(final double speed) // negative for back positive for forward
  {
    intake.set(speed);
  }

  /**
   * Stops the intake mechanism motor
   */
  public void stopSpin() {
    intake.set(0);
  }

  /**
   * Gets the last set value of the intake mechanism motor
   * @return A double representing the last set value of the intake mechanism motor
   */
  public double checkSpinSpeed() {
    return intake.getSpeed();
  }
}