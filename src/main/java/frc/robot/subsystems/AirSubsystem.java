/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AirSubsystem extends SubsystemBase {
  private final Compressor compressor = new Compressor(Constants.compressorPort);

  /**
   * Creates a new AirSubsystem.
   */
  public AirSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void start() {
    compressor.start();
  }

  public void stop() {
    compressor.stop();
  }

  public double getCurrent() {
    return compressor.getCompressorCurrent();
  }

}
