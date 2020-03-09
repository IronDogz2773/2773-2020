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
    public static final double speedIncrement = 0.012;
    public static final double rotationIncrement = 0.03;
    public static final double rotationSpeedCap = 0.7;
    public static final double movementSpeedCap = 0.7;
    public static final double requiredShooterRate = 1000;
    public static final double metersPerFoot = 0.3048;
    
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

    // PWM
    public static final int leftFrontPort = 0;
    public static final int rightFrontPort = 1;
    public static final int leftBackPort = 2;
    public static final int rightBackPort = 3;
    public static final int flyWheelPortA = 4;
    public static final int flyWheelPortB = 5;
    public static final int colorWheelPort = 6;
    public static final int intakePort = 7;
    public static final int conveyorRightPort = 8;
    public static final int conveyorLeftPort = 9;
    public static final int LEDport = 10;
    public static final int rightClimbPort = 11;
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
    public static final int spinButton = 1;
    public static final int visionButton = 7;
    public static final int gyroButton = 5;
    public static final int turnTestButton = 14;
    public static final int climbButton = 3;
}
