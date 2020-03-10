/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double pi = 3.1415926535;
    public static final double metersPerFoot = 0.3048;
    public static final double requiredShooterRate = 1000;
    public static final double indexerSpeedCap = 0.75;

    // Drive Constants
    public static final double ksVolts = 2.29;
    public static final double kvVoltSecondsPerMeter = 0.796 / metersPerFoot;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0192 /  metersPerFoot;
    public static final double kTrackWidthMeters = 1.972485 * metersPerFoot;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackWidthMeters);
    public static final double kPDriveVel = 0.615;
    public static final double distancePerPulse = .5 * Math.PI * metersPerFoot / 2048;
    

    // Auto Constants
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = .5;
    public static final double kRamseteB = 2.0;
    public static final double kRamseteZeta = 0.7;

    // Proximity
    public static final double microsecondsPerMeter = 1000000.0 / 2 / 340;
    public static final double minDistanceToFrontObsticle = 1 * microsecondsPerMeter;
    public static final int leftFrontProximitySensor = 0;
    public static final int rightFrontProximitySensor = 1;
    public static final int leftProximitySensor = 2;
    public static final int rightProximitySensor = 3;
    public static final boolean enableProximitySensor = false;

    // PWM
    public static final int leftWheelsPort = 11;
    public static final int rightWheelsPort = 2;
    public static final int flyWheelPortA = 14;
    public static final int flyWheelPortB = 5;
    public static final int colorWheelPort = 8;
    public static final int intakePort = 0;
    public static final int conveyorRightPort = 1;
    public static final int conveyorLeftPort = 10;
    public static final int LEDport = 4;
    public static final int rightClimbPort = 3;
    public static final int leftClimbPort = 12;

    // DIO
    public static final int leftEncoderPortA = 0;
    public static final int leftEncoderPortB = 1;
    public static final int rightEncoderPortA = 2;
    public static final int rightEncoderPortB = 3;
    public static final int shooterEncoderPortA = 4;
    public static final int shooterEncoderPortB = 5;

    // PCM
    public static final int compressorPort = 0;
    public static final int indexerForwardChannel = 1;
    public static final int indexerReverseChannel = 0;

    // USB
    public static final int joystickPort = 0;
    public static final int gamepadPort = 1;

    // Buttons
    public static final int spinInButton = 1;
    public static final int spinOutButton = 3;
    public static final int visionButton = 5;
    public static final int gyroButton = 7;
    public static final int turnTestButton = 14;
    public static final int climbButton = 2;
    public static final int indexerReverseButton = 6;
    public static final int lockButton = 3;

    // Axes
    public static final int leftJoystickY = 1;
    public static final int leftJoystickX = 0;
    public static final int rightJoystickY = 5;
    public static final int rightJoystickX = 4;
    public static final int leftTrigger = 2;
    public static final int rightTrigger = 3;

}
