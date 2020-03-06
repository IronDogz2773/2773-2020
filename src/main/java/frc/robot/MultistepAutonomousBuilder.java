package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveVisionCommand;
import frc.robot.commands.SingleShotCommand;
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
        else if (autoTrajectory == 7) {
            trajectoryFilePath = "path7";
        }
        else
            trajectoryFilePath = "";
    }

    public Command build()
    {
        AutonomousBuilder autonomousBuilder = new AutonomousBuilder(driveSubsystem, navigationSubsystem, trajectoryFilePath);
        Command firstPath = autonomousBuilder.build();
        DriveVisionCommand driveVisionCommand = new DriveVisionCommand(driveSubsystem, navigationSubsystem);
        SingleShotCommand singleShotCommand = new SingleShotCommand(shooterSubsystem, indexerSubsystem, 3);
        if(!trajectoryFilePath.equals("path6"))
        {
            if(trajectoryFilePath.equals("path1")){
                AutonomousBuilder secondAutonomousBuilder = new AutonomousBuilder(driveSubsystem, navigationSubsystem, "path2");
                Command secondPath = autonomousBuilder.build();
            }
            if(trajectoryFilePath.equals("path3")){
                AutonomousBuilder secondAutonomousBuilder = new AutonomousBuilder(driveSubsystem, navigationSubsystem, "path4");
                Command secondPath = autonomousBuilder.build();
            }
            if(trajectoryFilePath.equals("path4")){
                AutonomousBuilder secondAutonomousBuilder = new AutonomousBuilder(driveSubsystem, navigationSubsystem, "path5");
                Command secondPath = autonomousBuilder.build();
            }
        }
        return firstPath;
        //TODO figure out how to string multiple commands together
        //return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }
}