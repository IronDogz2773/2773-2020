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

/**
 * Represents a ball shooter mechanism and its related components
 * @author Ilya Dzialendzik
 * @author Yury Dzialendzik
 * @author irondogzrobotics@gmail.com
 */
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
    updateRate();
  }

  /**
   * Updates the rpm based on an encoder
   */
  private void updateRate() {
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

  /**
   * Spins the flywheel motors at a specified speed
   * @param speed The speed value of the flywheel motors between -1.0 and 1.0 to set
   */
  public void startSpin(final double speed) {
    flyWheelA.set(speed);
    flyWheelB.set(-speed);
  }

  /**
   * Stops the flywheel motors
   */
  public void stopSpin() {
    flyWheelA.set(0.0);
    flyWheelB.set(0.0);
  }

  /**
   * Gets the current spinRate as a decimal
   * @return A double representing the spin Rate (1 is 100%)
   */
  public double checkSpinRate() {
    return -rate / Constants.requiredShooterRate;
  }

  /**
   * Gets a value representing if the shooter is at rpm
   * @return A boolean representing whether the shooter motors are spinning at or above the designated rpm
   */
  public boolean atRate()
  {
    return atRate;
  }
  
}
