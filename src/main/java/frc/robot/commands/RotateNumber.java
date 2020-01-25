/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;

public class RotateNumber extends CommandBase {
  private ColorSubsystem Rotater;
  private final Joystick joy;
  String startingColor = Rotater.gettColor();
  int timesColorPassed = 0;
  boolean sameColor = false; 
    /**
     * Creates a new StartSpinCommand.
     */
    public RotateNumber(final ColorSubsystem Rotater, final Joystick joy) {
        this.joy = joy;
        addRequirements(Rotater);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timesColorPassed >= 6)
        {
            Rotater.stopR2D2();
        }
        else 
        {
            Rotater.doR2D2(0.5);
        }
        if (startingColor != Rotater.gettColor()) {
            sameColor = false;
        }
        if (startingColor == Rotater.gettColor() && !!(sameColor))  {
            timesColorPassed += 1;
            sameColor = true; 
        }
    }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
/*final Color detectedColor = m_colorSensor.getColor();
        String colorString;
        final ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
        
        if (match.color == kBlueTarget) {
            colorString = "Blue";
        } else if (match.color == kRedTarget) {
            colorString = "Red";
        } else if (match.color == kGreenTarget) {
            colorString = "Green";
        } else if (match.color == kYellowTarget) {
            colorString = "Yellow";
        } else {
            colorString = "Unknown";
        }
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", colorString); */

        /*   
  public final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  public final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  public final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  public final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  */