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

/**
 * Represents an air compressor and its related components
 * @author Tyler Graham
 */
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

  /**
   * Starts the air compressor
   */
  public void start() {
    compressor.start();
  }

  /** 
   * Stops the air compressor
   */
  public void stop() {
    compressor.stop();
  }

  /**
   * Gets the amperage used by the compressor
   * @return A double representing the current used in amps
   */
  public double getCurrent() {
    return compressor.getCompressorCurrent();
  }

}
