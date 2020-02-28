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
    public static double speedIncrement = 0.012;
    public static double rotationIncrement = 0.03;
    public static double rotationSpeedCap = 0.7;
    public static double movementSpeedCap = 0.7;
    public static double requiredShooterRate = 1000;
    static double metersPerFoot = 0.3048;
    
    // Drive Constants
    public static double ksVolts = 2.29;
    public static double kvVoltSecondsPerMeter = 0.796 / metersPerFoot;
    public static double kaVoltSecondsSquaredPerMeter = 0.0192 /  metersPerFoot;
    public static double kTrackWidthMeters = 1.972485 * metersPerFoot;
    public static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackWidthMeters);
    public static double kPDriveVel = 0.615;
    public static double distancePerPulse = .5 * Math.PI * metersPerFoot / 2048;
    

    // Auto Constants
    public static double kMaxSpeedMetersPerSecond = 1;
    public static double kMaxAccelerationMetersPerSecondSquared = .5;
    public static double kRamseteB = 2.0;
    public static double kRamseteZeta = 0.7;

    // PWM
    public static int leftFrontPort = 0;
    public static int rightFrontPort = 1;
    public static int leftBackPort = 2;
    public static int rightBackPort = 3;
    public static int flyWheelPortA = 4;
    public static int flyWheelPortB = 5;
    public static int colorWheelPort = 6;
    public static int intakePort = 7;
    public static int conveyorRightPort = 8;
    public static int conveyorLeftPort = 9;
    public static int LEDport = 10;
    public static int rightClimbPort = 11;
    public static int leftClimbPort = 12;

    // DIO
    public static int leftEncoderPortA = 0;
    public static int leftEncoderPortB = 1;
    public static int rightEncoderPortA = 2;
    public static int rightEncoderPortB = 3;
    public static int shooterEncoderPortA = 4;
    public static int shooterEncoderPortB = 5;

    // PCM
    public static int compressorPort = 0;
    public static int indexerForwardChannel = 1;
    public static int indexerReverseChannel = 0;

    // USB
    public static int joystickPort = 0;
    public static int gamepadPort = 1;

    // Buttons
    public static int spinButton = 1;
    public static int visionButton = 7;
    public static int gyroButton = 5;
    public static int turnTestButton = 14;
}
