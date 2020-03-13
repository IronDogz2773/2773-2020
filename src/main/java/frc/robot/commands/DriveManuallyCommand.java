package frc.robot.commands;

//import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveManuallyCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    private final NavigationSubsystem nav;
    public double speed;
    public double rotation;
    //private double speedCap;
    private final Joystick joystick;
    // private double speedAcceleration;
    // private double rotationAcceleration;

    public DriveManuallyCommand(final DriveSubsystem subsystem, NavigationSubsystem nav, final Joystick joystick) {
        driveSubsystem = subsystem;
        addRequirements(subsystem);
        this.nav = nav;
        this.joystick = joystick;
    }

    @Override
    public void initialize() {
        // Called at the beginning of each time command is used
        // speedAcceleration = .6;
        // rotationAcceleration = 1;
    }

    @Override
    public void execute() { // what the code does while the command is active
        /*
         * if(!driveSubsystem.driveState) { if(Math.abs(joy.getY()) > .15) { speed =
         * -joy.getY() * sAcc; if(sAcc < 1.00) sAcc += Constants.sInc; } else { sAcc =
         * 0; } if(Math.abs(joy.getZ()) > .15) { rot = joy.getZ() * rAcc; if(rAcc <
         * 1.00) rAcc += Constants.rInc; } else { rAcc = 0; }
         * driveSubsystem.manDrive(speed, rot); }
         */
        //speedCap = joystick.getRawAxis(Constants.joystickSlider) + 1.5;
        if (Math.abs(joystick.getY()) > .1) {
            speed = -joystick.getY();// * speedAcceleration * Constants.movementSpeedCap;
            /*
             * if(speedAcceleration < 1.00) speedAcceleration += Constants.speedIncrement;
             */
        } else {
            // speedAcceleration = 0.7;
            speed = 0;
        }
        if (Math.abs(joystick.getZ()) > .1) {
            rotation = joystick.getZ() / 1.2; // * rotationAcceleration * Constants.rotationSpeedCap;
            /*
             * if(rotationAcceleration < 1.00) rotationAcceleration +=
             * Constants.rotationIncrement;
             */
        } else {
            // rotationAcceleration = 1;
            rotation = 0;
        }

        // Don't drive if we close to something.
        if (!nav.tooClose() || speed < 0)
            driveSubsystem.rawDrive(speed, rotation);

        SmartDashboard.putNumber("Speed", speed);
        SmartDashboard.putNumber("Rotation", rotation);
        // driveSubsystem.accelerometer.getAccelerations();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(final boolean interrupted) {

    }

}