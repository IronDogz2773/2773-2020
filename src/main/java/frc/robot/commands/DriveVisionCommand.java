package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

public class DriveVisionCommand extends CommandBase {

    private static final double MAX_TURNING_SPEED = 0.55;
    private static final double TIMED_COMMAND_DURATION = 3.0;
    private final DriveSubsystem driveSubsystem;
    PIDController PIDcontrol = new PIDController(0.03, 0.02, 0);
    Timer time = new Timer();
    private double rotation;
    private boolean isTimed;
    private final NavigationSubsystem navigationSubsystem;

    public DriveVisionCommand(final DriveSubsystem subsystem, final NavigationSubsystem navigationSubsystem,
            boolean isTimed) {
        driveSubsystem = subsystem;
        addRequirements(subsystem);

        this.navigationSubsystem = navigationSubsystem;
        this.isTimed = isTimed;
    }

    @Override
    public void initialize() {
        driveSubsystem.driveState = true;
        PIDcontrol.setTolerance(1);
        PIDcontrol.setSetpoint(0);

        time.start();
    }

    @Override
    public void execute() {
        final double alpha = navigationSubsystem.getVisionAngle();
        final double e = PIDcontrol.calculate(navigationSubsystem.getGyroAngle());
        rotation = MathUtil.clamp(e, -MAX_TURNING_SPEED, MAX_TURNING_SPEED);

        driveSubsystem.rawDrive(0, rotation, false);
        driveSubsystem.driveState = true;
        SmartDashboard.putNumber("Alpha", alpha);
        SmartDashboard.putNumber("Rotation", rotation);

    }

    @Override
    public boolean isFinished() {
        if (time.get() >= TIMED_COMMAND_DURATION && isTimed) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(final boolean interrupted) {
        driveSubsystem.driveState = false;
    }

}