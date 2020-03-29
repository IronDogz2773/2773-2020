/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXL362;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import com.analog.adis16470.frc.ADIS16470_IMU;
/*import com.analog.adis16470.frc.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI.Port;*/

/**
 * Represents sensors associated with navigation and odometry
 * @author Ilya Dzialendzik
 * @author Yury Dzialendzik
 * @author Tyler Graham
 * @author irondogzrobotics@gmail.com
 */
public class NavigationSubsystem extends SubsystemBase {

  // Sensors
  private final ADXRS450_Gyro gyroscope = new ADXRS450_Gyro();
  private final ADXL362 accelerometer = new ADXL362(Accelerometer.Range.k2G);

  // Encoders
  private final Encoder leftEncoder = new Encoder(Constants.leftEncoderPortA, Constants.leftEncoderPortB);
  private final Encoder rightEncoder = new Encoder(Constants.rightEncoderPortA, Constants.rightEncoderPortB);

  private final NetworkTableEntry angleEntry;
  private final Rotation2d startRotation = new Rotation2d(0);
  private final Pose2d startPosition = new Pose2d(0, 0, startRotation);
  private final DifferentialDriveOdometry odometer = new DifferentialDriveOdometry(startRotation);
  private Pose2d currentPosition = new Pose2d(0, 0, startRotation);

  private double[] proximityDistances;

  private ProximitySensor proximity = new ProximitySensor();

  /**
   * Creates a new NavigationSubsystem and resets gyroscope.
   */
  public NavigationSubsystem() {
    if (gyroscope.isConnected()) {
      gyroscope.reset();
      gyroscope.calibrate();
    }
    accelerometer.setRange(Accelerometer.Range.k2G);

    final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    angleEntry = inst.getEntry("/angle");
    this.setDistancePerPulse(Constants.distancePerPulse);

    SmartDashboard.putData("Gyroscope", gyroscope);
    //SmartDashboard.putData("Accelerometer", accelerometer);
  }

  /**
   * Sets the distance per pulse of the left and right encoders
   * @param distancePerPulse The distance (in meters) per pulse of the encoders
   */
  public void setDistancePerPulse(double distancePerPulse) {
    leftEncoder.setDistancePerPulse(distancePerPulse);
    rightEncoder.setDistancePerPulse(distancePerPulse);
  }

  /**
   * Gets the angle of the target from the vision network table
   * @return A double representing the angle (in degrees) between the camera and the target
   */
  public double getVisionAngle() {
    return angleEntry.getDouble(0);
  }

  /**
   * Gets the current heading of the gyroscope
   * @return A double representing the currrent heading of the gyroscope (in degrees)
   */
  public double getGyroAngle() {
    return gyroscope.getAngle();
    // return imu.getAngle();
  }

  /**
   * Gets the current position of the robot
   * @return A Pose2d representing the current position of the robot
   */
  public Pose2d getCurrentPosition() {
    return currentPosition;
  }

  /**
   * Gets the starting position of the robot
   * @return A Pose2d representing the starting position of the robot
   */
  public Pose2d getStartPosition() {
    return startPosition;
  }

  /**
   * Resets the left and right encoders
   */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
   * Resets the gyroscope
   */
  public void resetGyro() {
    gyroscope.reset();
  }

  /** 
   * Resets the ododometer to the starting position and rotation
   */
  public void resetOdometer() {
    odometer.resetPosition(startPosition, startRotation);
  }

  /**
   * Gets a new Pose2d
   * @return A default Pose2d object
   */
  public Pose2d getPose() {
    return new Pose2d();
  }

  /** 
   * Gets a new DifferentialDriveWheelSpeeds 
   * @return A default DifferentialDriveWheelSpeeds object
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds();
  }

  /**
   * Gets the current heading of the gyroscope in degrees (0 to 360)
   * @return a double representing the current heading of the gyroscope (in degrees 0 to 360)
   * @deprecated
   */
  public double getHeading() {
    return Math.IEEEremainder(gyroscope.getAngle(), 360); // * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Uses ultrasonic sensors to check if the robot is too close to an obstacle
   * @return A boolean representing if the robot is too close to an obstacle
   */
  public boolean tooClose() {
    return proximityDistances[Constants.leftFrontProximitySensor] <= Constants.minDistanceToFrontObsticle
        || proximityDistances[Constants.rightFrontProximitySensor] <= Constants.minDistanceToFrontObsticle;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var gyroAngle = Rotation2d.fromDegrees(-gyroscope.getAngle());
    currentPosition = odometer.update(gyroAngle, -leftEncoder.getDistance(), rightEncoder.getDistance());
    //SmartDashboard.putNumber("Left Encoder", -leftEncoder.getDistance());
    //SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());

    proximityDistances = proximity.getDistances();
    //SmartDashboard.putNumberArray("Distances", proximityDistances);
  }
}
