package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

    private static final double MOTOR_VOLTS = 12.0;

    // Motors
    private final PWMVictorSPX leftWheels = new PWMVictorSPX(Constants.leftWheelsPort);
    private final PWMVictorSPX rightWheels = new PWMVictorSPX(Constants.rightWheelsPort);

    // Speed controller groups
    private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftWheels);
    private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightWheels);

    // Differential drive
    private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    public boolean driveState = false; // when true vision is running

    public DriveSubsystem() {

        drive.setDeadband(.1);
    }

    public void rawDrive(final double speed, final double rotation) {
        drive.arcadeDrive(speed, rotation);
    }

    public void rawDrive(final double speed, final double rotation, final boolean accel) {
        drive.arcadeDrive(speed, rotation, accel);
    }

    public void rotate(final double rotation) {
        drive.arcadeDrive(0, rotation, false);
    }

    public void stop() {
        drive.tankDrive(0, 0);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        drive.tankDrive(leftVolts / MOTOR_VOLTS, rightVolts / MOTOR_VOLTS, false);
    }
}