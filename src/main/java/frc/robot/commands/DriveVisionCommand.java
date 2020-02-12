package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

public class DriveVisionCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    PIDController PIDcontrol = new PIDController(0.03, 0.02, 0);
    private double rotation;
    private double target;
    private final NavigationSubsystem nav;

    public DriveVisionCommand(final DriveSubsystem subsystem, final NavigationSubsystem nav) {
        driveSubsystem = subsystem;
        addRequirements(subsystem);

        this.nav = nav;
    }

    @Override
    public void initialize() {
        driveSubsystem.driveState = true;
        PIDcontrol.setTolerance(1);
        PIDcontrol.setSetpoint(0);

        target = 10;
        PIDcontrol.setSetpoint(target);
    }

    @Override
    public void execute() { // what the code does while the command is active
        final double alpha = nav.getVisionAngle();
        SmartDashboard.putNumber("Target", target);
        final double e = PIDcontrol.calculate(nav.getGyroAngle());
        rotation = MathUtil.clamp(e, -0.55, 0.55);

        driveSubsystem.rawDrive(0, rotation, false);
        driveSubsystem.driveState = true;
        SmartDashboard.putNumber("Alpha", alpha);
        SmartDashboard.putNumber("Rotation", rotation);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(final boolean interrupted) {
        driveSubsystem.driveState = false;
    }

}