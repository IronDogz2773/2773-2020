package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveVisionCommand;
import frc.robot.commands.SingleShotCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MultistepAutonomousBuilder {
    private final DriveSubsystem driveSubsystem;
    private final NavigationSubsystem navigationSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private int autoTrajectory;
    private String firstTrajectoryFilePath;
    private String secondTrajectoryFilePath;

    public MultistepAutonomousBuilder(DriveSubsystem driveSubsystem, NavigationSubsystem navigationSubsystem,
            IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem, int autoTrajectory) {
        this.driveSubsystem = driveSubsystem;
        this.navigationSubsystem = navigationSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.autoTrajectory = autoTrajectory;
        setAutoFilePath();
    }

    public void setAutoFilePath() {
        firstTrajectoryFilePath = "";
        if (autoTrajectory == 0) {
            firstTrajectoryFilePath = "path1";
        }
        else if (autoTrajectory == 1) {
            firstTrajectoryFilePath = "path3";
        }
        else if (autoTrajectory == 2){
            firstTrajectoryFilePath = "path5";
        }
        else if (autoTrajectory == 3){
            firstTrajectoryFilePath = "path7";
        }
    }

    public Command build()
    {
        AutonomousBuilder firstAutonomousBuilder = new AutonomousBuilder(driveSubsystem, navigationSubsystem, firstTrajectoryFilePath);
        Command firstPath = firstAutonomousBuilder.build(); //first Ramsete command
        DriveVisionCommand driveVisionCommand = new DriveVisionCommand(driveSubsystem, navigationSubsystem, true);
        SingleShotCommand singleShotCommand = new SingleShotCommand(shooterSubsystem, indexerSubsystem, 3);

        if(!firstTrajectoryFilePath.equals("path6"))
        {
            if(firstTrajectoryFilePath.equals("path1")){
                secondTrajectoryFilePath = "path2";
            }
            else if(firstTrajectoryFilePath.equals("path3")){
                secondTrajectoryFilePath = "path4";
            }
            else if(firstTrajectoryFilePath.equals("path5")){
                secondTrajectoryFilePath = "path6";
            }
            AutonomousBuilder secondAutonomousBuilder = new AutonomousBuilder(driveSubsystem, navigationSubsystem, secondTrajectoryFilePath);
            Command secondPath = secondAutonomousBuilder.build();
            SequentialCommandGroup autonomousCommands = new SequentialCommandGroup(firstPath, driveVisionCommand, singleShotCommand, secondPath);
            return autonomousCommands;
        }
        else
        {
            return firstPath;
        }
        //TODO figure out how to string multiple commands together
        //return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
    }
}