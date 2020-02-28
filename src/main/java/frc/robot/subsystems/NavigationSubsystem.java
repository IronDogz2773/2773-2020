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

public class NavigationSubsystem extends SubsystemBase {

  // Sensors
  private final ADXRS450_Gyro gyroscope = new ADXRS450_Gyro();
  //private final ADIS16470_IMU imu = new ADIS16470_IMU(ADIS16470_IMU.IMUAxis.kY, SPI.Port.kMXP, ADIS16470_IMU.ADIS16470CalibrationTime._256ms);
  private final ADXL362 accelerometer = new ADXL362(Accelerometer.Range.k2G);

  // Encoders
  private final Encoder leftEncoder = new Encoder(Constants.leftEncoderPortA, Constants.leftEncoderPortB);
  private final Encoder rightEncoder = new Encoder(Constants.rightEncoderPortA, Constants.rightEncoderPortB);

  private final NetworkTableEntry angleEntry;
  private final Rotation2d startRotation = new Rotation2d(0);
  private final Pose2d startPosition = new Pose2d(0, 0, startRotation);
  private final DifferentialDriveOdometry odometer = new DifferentialDriveOdometry(startRotation);
  private Pose2d currentPosition = new Pose2d(0, 0, startRotation);

  private ProximitySensor proximity = new ProximitySensor();
  /**
   * Creates a new NavigationSubsystem.
   */
  public NavigationSubsystem() {
    if (gyroscope.isConnected()) {
      gyroscope.reset();
      gyroscope.calibrate();
    }
    accelerometer.setRange(Accelerometer.Range.k2G);

    final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    angleEntry = inst.getEntry("/angle");

    SmartDashboard.putData("Gyroscope", gyroscope);
    SmartDashboard.putData("Accelerometer", accelerometer);
  }

  public void setDistancePerPulse(double distancePerPulse)
  {
    leftEncoder.setDistancePerPulse(distancePerPulse);
    rightEncoder.setDistancePerPulse(distancePerPulse);
  }

  public double getVisionAngle() {
    return angleEntry.getDouble(0);
  }

  public double getGyroAngle() {
    return gyroscope.getAngle();
    //return imu.getAngle();
  }

  public Pose2d getCurrentPosition()
  {
    return currentPosition;
  }

  public Pose2d getStartPosition()
  {
    return startPosition;
  }

  public void resetEncoders()
  {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  public void resetGyro() {
    gyroscope.reset();
  }
  
  public void resetOdometer(){
    odometer.resetPosition(startPosition, startRotation);
  }
  public Pose2d getPose() {
    return new Pose2d();
  }
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    var gyroAngle = Rotation2d.fromDegrees(-gyroscope.getAngle());
    currentPosition = odometer.update(gyroAngle, -leftEncoder.getDistance(), rightEncoder.getDistance());
    SmartDashboard.putNumber("Left Encoder", -leftEncoder.getDistance());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.getDistance());

    double[] d = proximity.getDistances();
    SmartDashboard.putNumberArray("Distances", d);
  }
}
