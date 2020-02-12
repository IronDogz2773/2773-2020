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
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.analog.adis16470.frc.ADIS16470_IMU;
/*import com.analog.adis16470.frc.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI.Port;*/

public class NavigationSubsystem extends SubsystemBase {

  // Sensors
  private final ADXRS450_Gyro gyroscope = new ADXRS450_Gyro();
  private final ADIS16470_IMU imu = new ADIS16470_IMU(ADIS16470_IMU.IMUAxis.kY, SPI.Port.kMXP, ADIS16470_IMU.ADIS16470CalibrationTime._256ms);
  private final ADXL362 accelerometer = new ADXL362(Accelerometer.Range.k2G);

  private final NetworkTableEntry angleEntry;

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
    SmartDashboard.putData("NewCoolIMU", imu);
  }

  public double getVisionAngle() {
    return angleEntry.getDouble(0);
  }

  public double getGyroAngle() {
    return gyroscope.getAngle();
    //return imu.getAngle();
  }

  public void resetGyro() {
    gyroscope.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
