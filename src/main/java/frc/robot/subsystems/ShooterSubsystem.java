/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private final Spark flyWheelA = new Spark(Constants.flyWheelPortA);
  private final Spark flyWheelB = new Spark(Constants.flyWheelPortB);

  private final Encoder shooterEncoder = new Encoder(Constants.shooterEncoderPortA, Constants.shooterEncoderPortB);
  private double rate;
  private boolean atRate;

  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    rate = 0;
    atRate = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rate = shooterEncoder.getRate();
    if(rate >= Constants.requiredShooterRate)
    {
      atRate = true;
    }
    else
    {
      atRate = false;
    }
  }

  public void startSpin(final double speed) {
    flyWheelA.set(speed);
    flyWheelB.set(-speed);
  }

  public void stopSpin() {
    flyWheelA.set(0.0);
    flyWheelB.set(0.0);
  }

  // encoder tells me the speed is right if true
  public double checkSpinSpeed() {
    return flyWheelA.get();
  }

  public boolean atRate()
  {
    return atRate;
  }

  public void send() {
  }
}
