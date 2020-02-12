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
import frc.robot.Constants;

public class IndexerSubsystem extends SubsystemBase {
  private final Spark conveyorRight = new Spark(Constants.conveyorRightPort);
  private final Spark conveyorLeft = new Spark(Constants.conveyorLeftPort);
  private final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(Constants.indexerForwardChannel,
      Constants.indexerReverseChannel);

  /**
   * Creates a new IndexerSubsystem.
   */
  public IndexerSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void startConveyorSpin(final double speed) {
    conveyorRight.set(speed); // one turns clockwise, the other turns counterclockwise
    conveyorLeft.set(-speed);
  }

  public void stopConveyorSpin() {
    conveyorRight.set(0); // one turns clockwise, the other turns counterclockwise
    conveyorLeft.set(0);
  }

  public void lock(final boolean locked) // kForward for on, kReverse for off
  {
    if (locked)
      doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    else
      doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public DoubleSolenoid.Value getSolenoidValue() {
    return doubleSolenoid.get();
  }
}
