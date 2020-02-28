/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.commands.ClimbControllerCommand;
import frc.robot.commands.CompressorControlCommand;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.DriveVisionCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeSpinCommand;
import frc.robot.commands.LEDControlCommand;
import frc.robot.commands.ResetGyroscopeCommand;
import frc.robot.commands.StartSpinCommand;
import frc.robot.commands.TurnDegreesCommand;
import frc.robot.subsystems.AirSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private static Joystick joystick = new Joystick(Constants.joystickPort);
  private static Joystick gamepad = new Joystick(Constants.gamepadPort);
  private final PowerDistributionPanel powerDistributionPanel = new PowerDistributionPanel();
  private final UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();

  // Subsystems
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final NavigationSubsystem navigationSubsystem = new NavigationSubsystem();
  private final AirSubsystem airSubsystem = new AirSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

  // Commands
  private final DriveManuallyCommand driveManuallyCommand = new DriveManuallyCommand(driveSubsystem, joystick);
  private final StartSpinCommand startSpinCommand = new StartSpinCommand(shooterSubsystem);
  private final IntakeSpinCommand intakeSpinCommand = new IntakeSpinCommand(intakeSubsystem, gamepad);
  private final DriveVisionCommand visionCommand = new DriveVisionCommand(driveSubsystem, navigationSubsystem);
  private final ResetGyroscopeCommand resetGyroscopeCommand = new ResetGyroscopeCommand(navigationSubsystem);
  private final CompressorControlCommand compressorControlCommand = new CompressorControlCommand(airSubsystem);
  private final IndexerCommand indexerCommand = new IndexerCommand(indexerSubsystem, gamepad);
  private final TurnDegreesCommand turn90Command = new TurnDegreesCommand(driveSubsystem, navigationSubsystem, 90);
  private final LEDControlCommand ledControlCommand = new LEDControlCommand(ledSubsystem);
  private final ClimbControllerCommand climbControllerCommand = new ClimbControllerCommand(climbSubsystem, gamepad);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    setShuffleboardVals();
    driveSubsystem.setDefaultCommand(driveManuallyCommand);
    intakeSubsystem.setDefaultCommand(intakeSpinCommand);
    airSubsystem.setDefaultCommand(compressorControlCommand);
    indexerSubsystem.setDefaultCommand(indexerCommand);
    ledSubsystem.setDefaultCommand(ledControlCommand);
    climbSubsystem.setDefaultCommand(climbControllerCommand);
    camera.setResolution(160, 120);
    camera.setFPS(15);

    initPIDTable();

    // shooterSubsystem.setDefaultCommand(spinCmd);
    // TODO give remaining subsystems default commands
  }

  private void initPIDTable() {
    final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    final NetworkTable pidTable = inst.getTable("PID");
    if(pidTable.getEntry("P").getDouble(0) != 0) 
      return;
    pidTable.getEntry("P").forceSetNumber(.03);
    pidTable.getEntry("I").forceSetNumber(.00);
    pidTable.getEntry("D").forceSetNumber(.00);
    pidTable.getEntry("Test angle").forceSetNumber(90);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final JoystickButton spinButton = new JoystickButton(joystick, Constants.spinButton);
    spinButton.whenHeld(startSpinCommand, true);
    final JoystickButton visionButton = new JoystickButton(joystick, Constants.visionButton);
    visionButton.whenHeld(visionCommand, true);
    final JoystickButton gyroButton = new JoystickButton(joystick, Constants.gyroButton);
    gyroButton.whenHeld(resetGyroscopeCommand, true);
    final JoystickButton turnButton = new JoystickButton(joystick, Constants.turnTestButton);
    turnButton.whenHeld(turn90Command, true);
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                             Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config
    );

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
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

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> driveSubsystem.tankDriveVolts(0, 0));
  }

  private void setShuffleboardVals() {
    SmartDashboard.putBoolean("Shooter", false);
    SmartDashboard.putNumber("Speed", driveManuallyCommand.speed);
    SmartDashboard.putNumber("Rotation", driveManuallyCommand.rotation);
    SmartDashboard.putData("Power Distribution Panel", powerDistributionPanel);

  }
}
