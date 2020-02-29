package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

public class AutonomousBuilder {
    String trajectoryJSON;
    Trajectory trajectory;
    DifferentialDriveVoltageConstraint autoVoltageConstraint;
    DriveSubsystem driveSubsystem;
    NavigationSubsystem navigationSubsystem;
    RamseteCommand ramseteCommand;
    String trajectoryFilePath;

    public AutonomousBuilder(DriveSubsystem driveSubsystem, NavigationSubsystem navigationSubsystem, String filePath){
        this.driveSubsystem = driveSubsystem;
        this.navigationSubsystem = navigationSubsystem;
        this.trajectoryFilePath = filePath;
        setVoltageConstraint();
        setTrajectory(trajectoryFilePath);
        setRamsete();
    }

    private void setVoltageConstraint()
    {
        autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
                                       Constants.kDriveKinematics, 10);
    }

    private void setTrajectory(String filePath)
    {
        trajectoryJSON = filePath;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } 
        catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
    }

    private void setRamsete()
    {
        ramseteCommand = new RamseteCommand(
            trajectory,
            navigationSubsystem::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            navigationSubsystem::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0),
            new PIDController(Constants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            driveSubsystem::tankDriveVolts,
            driveSubsystem
        );
    }

    public RamseteCommand getRamseteCommand()
    {
        return ramseteCommand;
    }
}