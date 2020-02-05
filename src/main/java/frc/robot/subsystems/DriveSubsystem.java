package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXL362;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;

public class DriveSubsystem extends SubsystemBase {

    //Motors
    private final Spark FL = new Spark(Constants.leftFrontPort); 
    private final Spark FR = new Spark(Constants.rightFrontPort);
    private final Spark BL = new Spark(Constants.leftBackPort);
    private final Spark BR = new Spark(Constants.rightBackPort);

    //Speed controller groups
    private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(FL, BL); 
    private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(FR, BR);

    //Differential drive
    private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    //Sensors
    public final ADXRS450_Gyro gyroscope = new ADXRS450_Gyro();
    public final ADXL362 accelerometer = new ADXL362(Accelerometer.Range.k2G);

    public boolean driveState = false; //when true vision is running

    public DriveSubsystem(){
        if(gyroscope.isConnected())
        {
            gyroscope.reset();
            gyroscope.calibrate();
        }
        accelerometer.setRange(Accelerometer.Range.k2G);
    }

    public void rawDrive(final double speed, final double rotation) {
        drive.arcadeDrive(speed, rotation);
    }

    public void rawDrive(final double speed, final double rotation, final boolean accel)
    {
        drive.arcadeDrive(speed, rotation, accel);
    }
}