package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
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
    String trajectoryFilePath;

    public AutonomousBuilder(DriveSubsystem driveSubsystem, NavigationSubsystem navigationSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.navigationSubsystem = navigationSubsystem;
        setVoltageConstraint();
        setDemoTrajectory();
    }

    public AutonomousBuilder(DriveSubsystem driveSubsystem, NavigationSubsystem navigationSubsystem, String filePath) {
        this.driveSubsystem = driveSubsystem;
        this.navigationSubsystem = navigationSubsystem;
        this.trajectoryFilePath = filePath;
        setVoltageConstraint();
        setTrajectoryPath(trajectoryFilePath);
    }

    private void setVoltageConstraint()

    {
        autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics,
                10);
    }

    private void setDemoTrajectory()
    {
        TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                                                       Constants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);
        trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        new Pose2d(3, 0, new Rotation2d(0)),
        config);
    }

    private void setTrajectoryPath(String filePath) {
        trajectoryJSON = filePath;
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
    }

    public Command build() {
        var ramseteCommand = new RamseteCommand(trajectory, navigationSubsystem::getPose,
            new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.ksVolts, 
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics, 
            navigationSubsystem::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0), 
            new PIDController(Constants.kPDriveVel, 0, 0),
            driveSubsystem::tankDriveVolts, driveSubsystem);
        return ramseteCommand;
    }
}