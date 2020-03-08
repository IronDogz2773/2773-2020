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

    public final static int LEFT = 0;
    public final static int MIDDLE = 1;
    public final static int RIGHT = 2;
    public final static int RIGHT_RETREAT = 3;

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
        switch (autoTrajectory) {
        case LEFT:
            firstTrajectoryFilePath = "output/Left1.wpilib.json";
            secondTrajectoryFilePath = "output/Left2.wpilib.json";
            break;
        case RIGHT:
            firstTrajectoryFilePath = "output/Right1.wpilib.json";
            secondTrajectoryFilePath = "output/Right2.wpilib.json";
            break;
        case MIDDLE:
            firstTrajectoryFilePath = "output/Middle1.wpilib.json";
            secondTrajectoryFilePath = "output/Middle2.wpilib.json";
            break;
        case RIGHT_RETREAT:
            firstTrajectoryFilePath = "output/RightRetreat.wpilib.json";
            secondTrajectoryFilePath = "";
            break;
        default:
            firstTrajectoryFilePath = "";
            secondTrajectoryFilePath = "";
            break;

        }
    }

    public Command build() {
        AutonomousBuilder firstAutonomousBuilder = new AutonomousBuilder(driveSubsystem, navigationSubsystem,
                firstTrajectoryFilePath);
        Command firstPath = firstAutonomousBuilder.build(); // first Ramsete command
        DriveVisionCommand driveVisionCommand = new DriveVisionCommand(driveSubsystem, navigationSubsystem, true);
        SingleShotCommand singleShotCommand = new SingleShotCommand(shooterSubsystem, indexerSubsystem, 3);

        if (!secondTrajectoryFilePath.equals("")) {
            AutonomousBuilder secondAutonomousBuilder = new AutonomousBuilder(driveSubsystem, navigationSubsystem,
                    secondTrajectoryFilePath);
            Command secondPath = secondAutonomousBuilder.build();
            SequentialCommandGroup autonomousCommands = new SequentialCommandGroup(firstPath, driveVisionCommand,
                    singleShotCommand, secondPath);
            return autonomousCommands;
        } else {
            return firstPath;
        }
    }
}