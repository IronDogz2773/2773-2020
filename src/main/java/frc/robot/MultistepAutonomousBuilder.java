package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class MultistepAutonomousBuilder {
    private final DriveSubsystem driveSubsystem;
    private final NavigationSubsystem navigationSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private int autoTrajectory;
    private String trajectoryFilePath;

    public MultistepAutonomousBuilder(DriveSubsystem driveSubsystem, NavigationSubsystem navigationSubsystem,
            IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem, int autoTrajectory) {
        this.driveSubsystem = driveSubsystem;
        this.navigationSubsystem = navigationSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.autoTrajectory = autoTrajectory;
        //TODO set autoTrajectory based on sendable chooser
        setAutoFilePath();
    }

    public void setAutoFilePath() {
        trajectoryFilePath = "";
        if (autoTrajectory == 0) {
            trajectoryFilePath = "path1";
        }
        else if (autoTrajectory == 1) {
            trajectoryFilePath = "path2";
        }
        else if (autoTrajectory == 3) {
            trajectoryFilePath = "path3";
        }
        else if (autoTrajectory == 4) {
            trajectoryFilePath = "path4";
        }
        else if (autoTrajectory == 5) {
            trajectoryFilePath = "path5";
        }
        else if (autoTrajectory == 6) {
            trajectoryFilePath = "path6";
        }
        else
            trajectoryFilePath = "";
    }

    /*public Command build()
    {
        return new CommandBase command;
        //return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }*/
}