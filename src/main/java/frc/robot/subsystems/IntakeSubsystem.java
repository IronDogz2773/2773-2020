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

  public void startSpin(final double speed) // negative for back positive for forward
  {
    intake.set(speed);
  }

  public void stopSpin() {
    intake.set(0);
  }

  public double checkSpinSpeed() {
    return intake.getSpeed();
  }

  public void intakeMax() {
  }
  // TODO find the max amount of area for the balls

  public void intakeStop() {
  }
  // TODO if there are no balls then the shooter cannot work
}