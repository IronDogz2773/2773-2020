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

public class ShooterSubsystem extends SubsystemBase {

  private final Spark flyWheel = new Spark(Constants.flyWheelPort);

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startSpin(final double speed) {
    flyWheel.set(speed);
  }

  public void stopSpin() {
    flyWheel.set(0.0);
  }

  // encoder tells me the speed is right if true
  public double checkSpinSpeed() {
    return flyWheel.get();
  }

  public void send() {
  }
}
