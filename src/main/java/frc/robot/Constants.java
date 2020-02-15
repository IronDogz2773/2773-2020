/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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

    // PWM
    public static int leftFrontPort = 0;
    public static int rightFrontPort = 1;
    public static int leftBackPort = 2;
    public static int rightBackPort = 3;
    public static int flyWheelPort = 4;
    public static int colorWheelPort = 5;
    public static int intakePort = 6;
    public static int conveyorRightPort = 7;
    public static int conveyorLeftPort = 8;

    // DIO
    public static int leftEncoderPortA = 0;
    public static int leftEncoderPortB = 1;
    public static int rightEncoderPortA = 2;
    public static int rightEncoderPortB = 3;

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
