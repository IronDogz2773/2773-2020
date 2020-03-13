/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
  
  private final Spark LED = new Spark(Constants.LEDport);
  /**
   * Creates a new LEDSubsystem.
   */
  public LEDSubsystem() {
    LED.set(0);
  }

  public void setLED(double val)
  {
    LED.set(val);
  }

  public double getLED()
  {
    return LED.get();
  }

  public String getTeam()
  {
    NetworkTable fms = NetworkTableInstance.getDefault().getTable("FMSInfo");
    boolean team = fms.getEntry("IsRedAlliance").getBoolean(false);
    if(team)
      return "blue";
    else if(!team)
      return "red";
    else  
      return "teamError"; 
  }

  public void switchToClimb()
  {
    setLED(0.91);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
