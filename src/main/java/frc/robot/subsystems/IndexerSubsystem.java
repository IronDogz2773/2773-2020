/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;

/**
 * Represents a power cell indexer and its related components
 * @author Tyler Graham
 * @author Yury Dzialendzik
 * @author irondogz@gmail.com
 */
public class IndexerSubsystem extends SubsystemBase {
  private final Spark conveyorRight = new Spark(Constants.conveyorRightPort);
  private final Spark conveyorLeft = new Spark(Constants.conveyorLeftPort);
  private final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(Constants.indexerForwardChannel,
      Constants.indexerReverseChannel);
  private boolean lockState;

  /**
   * Creates a new IndexerSubsystem with a default lockState of false (solenoid locking mechanism is not activated)
   */
  public IndexerSubsystem() {
    lockState = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the speed of the indexer conveyor motors
   * @param speed The speed value of the conveyor motors between -1.0 and 1.0 to set
   */
  public void setConveyorSpeed(double speed) {
    speed = MathUtil.clamp(speed, -Constants.indexerSpeedCap, Constants.indexerSpeedCap);
    conveyorRight.set(-speed); // one turns clockwise, the other turns counterclockwise
    conveyorLeft.set(speed);
  }

  /**
   * Stops the conveyor motors
   */
  public void stopConveyor() {
    conveyorRight.set(0); // one turns clockwise, the other turns counterclockwise
    conveyorLeft.set(0);
  }

  /**
   * Controls the state of the solenoid locking mechanism
   * @param locked If set, the solenoid locking mechanism will activate
   */
  public void lock(final boolean locked) // kForward for on, kReverse for off
  {
    if(locked == lockState)
    return;
    if (locked)
      doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    else
      doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    lockState = locked;
  }

  /**
   * Gets the current value of the solenoid locking mechanism
   * @return A DoubleSolenoid.Value representing the current state of the solenoid locking mechanism
   */
  public DoubleSolenoid.Value getSolenoidValue() {
    return doubleSolenoid.get();
  }
}
   