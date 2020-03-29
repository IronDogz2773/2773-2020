package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Represents a drivetrain and its related components
 * @author Ilya Dzialendzik
 * @author Yury Dzialendzik
 * @author Tyler Graham
 * @author irondogzrobotics@gmail.com
 */
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

    /**
     * Creates a new DriveSubsystem with a default deadband of 0.1
     */
    public DriveSubsystem() {

        drive.setDeadband(.1); 
    }

    /**
     * Drives the robot at a specified speed and rotation
     * @param speed The speed value of the drivetrain between -1.0 and 1.0 to set
     * @param rotation The rotation value of the drivetrain between -1.0 and 1.0 to set
     */
    public void rawDrive(final double speed, final double rotation) {
        drive.arcadeDrive(speed, rotation);
    }

    /**
     * Drives the robot at a specified speed and rotation
     * @param speed The speed value of the drivetrain between -1.0 and 1.0 to set
     * @param rotation The rotation value of the drivetrain between -1.0 and 1.0 to set
     * @param accel If set, decreases the input sensitivity at low speeds
     */
    public void rawDrive(final double speed, final double rotation, final boolean accel) {
        drive.arcadeDrive(speed, rotation, accel);
    }

    /**
     * Rotates the robot at a specified speed
     * @param rotation The rotation value of the drivetrain between -1.0 and 1.0 to set
     */
    public void rotate(final double rotation) {
        drive.arcadeDrive(0, rotation, false);
    }

    /**
     * Stops the robot
     */
    public void stop() {
        drive.tankDrive(0, 0);
    }

    /**
     * Drives the robot in a tank like fashion
     * @param leftVolts The speed of the left motor in volts
     * @param rightVolts The speed of the right motor in volts
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        drive.tankDrive(leftVolts / MOTOR_VOLTS, rightVolts / MOTOR_VOLTS, false);
    }
}