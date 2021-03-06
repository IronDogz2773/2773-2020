/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.I2C;

/**
 * Represents a color wheel spinner and its related components
 * @author David Finch
 * @author irondgoz@gmail.com
 */
public class ColorSubsystem extends SubsystemBase {
    private final Spark Tyler = new Spark(Constants.colorWheelPort);
    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    public final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    public final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    public final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
    public final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
    private final ColorMatch m_colorMatcher = new ColorMatch();
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

    /**
     * Creates a new ColorSubsystem
     */
    public ColorSubsystem() {
        m_colorMatcher.addColorMatch(kBlueTarget);
        m_colorMatcher.addColorMatch(kGreenTarget);
        m_colorMatcher.addColorMatch(kRedTarget);
        m_colorMatcher.addColorMatch(kYellowTarget);
    }

    @Override
    public void periodic() {
    }

    /**
     * Spins the color wheel motor at a specified speed
     * @param speed The speed value of the color wheel motor between -1.0 and 1.0 to set
     */
    public void doR2D2(final double speed) {
        Tyler.set(speed);
    }

    /**
     * Stops the color wheel motor
     */
    public void stopR2D2() {
        Tyler.set(0.0);
    }

    /**
     * Gets the current color detected by the REV Robotics Color Sensor V3
     * @return A Color representing the current color detected by the color sensor
     */
    public String gettColor() {
        final Color detectedColor = m_colorSensor.getColor();
        final ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (match.color == kBlueTarget) {
            return "Blue";
        } else if (match.color == kRedTarget) {
            return "Red";
        } else if (match.color == kGreenTarget) {
            return "Green";
        } else if (match.color == kYellowTarget) {
            return "Yellow";
        } else {
            return "Unknown";
        }
    }

    /**
     * Gets the designated match color
     * @return A String representing the color of the match
     * @deprecated 
     */
    public String getFieldColor() {
        return "";
    }
}
