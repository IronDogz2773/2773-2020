package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    public boolean driveState = false; //when true vision is running

    public DriveSubsystem(){
        //enter constructor properties here
    }

    public void manDrive(double speed, double rot)
    {
        drive.arcadeDrive(speed, rot);
    }
}