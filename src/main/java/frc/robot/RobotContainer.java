/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ClimbControllerCommand;
import frc.robot.commands.CompressorControlCommand;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.DriveVisionCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeSpinCommand;
import frc.robot.commands.LEDControlCommand;
import frc.robot.commands.MultiShotCommand;
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
  SendableChooser<Integer> autoChooser = new SendableChooser<>();

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
  private final DriveManuallyCommand driveManuallyCommand = new DriveManuallyCommand(driveSubsystem,
      navigationSubsystem, joystick);
  private final StartSpinCommand startSpinCommand = new StartSpinCommand(shooterSubsystem, gamepad);
  private final IntakeSpinCommand intakeSpinInCommand = new IntakeSpinCommand(intakeSubsystem, -1.0);
  private final IntakeSpinCommand intakeSpinOutCommand = new IntakeSpinCommand(intakeSubsystem, 1.0);
  private final DriveVisionCommand visionCommand = new DriveVisionCommand(driveSubsystem, navigationSubsystem, false);
  private final ResetGyroscopeCommand resetGyroscopeCommand = new ResetGyroscopeCommand(navigationSubsystem);
  private final CompressorControlCommand compressorControlCommand = new CompressorControlCommand(airSubsystem);
  private final IndexerCommand indexerCommand = new IndexerCommand(indexerSubsystem, gamepad);
  private final TurnDegreesCommand turn90Command = new TurnDegreesCommand(driveSubsystem, navigationSubsystem, 90);
  private final LEDControlCommand ledControlCommand = new LEDControlCommand(ledSubsystem);
  private final ClimbControllerCommand climbControllerCommand = new ClimbControllerCommand(climbSubsystem, ledSubsystem, gamepad);
  private final MultiShotCommand multiShotCommand = new MultiShotCommand(shooterSubsystem, indexerSubsystem, 3);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    addSendableChooserOptions();
    setShuffleboardVals();
    driveSubsystem.setDefaultCommand(driveManuallyCommand);
    airSubsystem.setDefaultCommand(compressorControlCommand);
    indexerSubsystem.setDefaultCommand(indexerCommand);
    ledSubsystem.setDefaultCommand(ledControlCommand);
    shooterSubsystem.setDefaultCommand(startSpinCommand);
    camera.setResolution(160, 120);
    camera.setFPS(15);

    initPIDTable();
  }

  private void initPIDTable() {
    final NetworkTableInstance inst = NetworkTableInstance.getDefault();
    final NetworkTable pidTable = inst.getTable("PID");
    if (pidTable.getEntry("P").getDouble(0) != 0)
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
    final JoystickButton gyroButton = new JoystickButton(joystick, Constants.gyroButton);
    gyroButton.whenHeld(resetGyroscopeCommand, true);
    final JoystickButton turnButton = new JoystickButton(joystick,Constants.turnTestButton); 
    turnButton.whenHeld(turn90Command, true);
    final JoystickButton climbButton = new JoystickButton(gamepad, Constants.climbButton);
    climbButton.whenPressed(climbControllerCommand);
    final JoystickButton spinInButton = new JoystickButton(joystick, Constants.spinInButton);
    spinInButton.whenHeld(intakeSpinInCommand, true);
    final JoystickButton spinOutButton = new JoystickButton(joystick, Constants.spinOutButton);
    spinOutButton.whenHeld(intakeSpinOutCommand);
    final JoystickButton visionButton = new JoystickButton(joystick, Constants.visionButton);
    visionButton.whenHeld(visionCommand, true);
    final JoystickButton multiShotButton = new JoystickButton(joystick, Constants.multiShotButton);
    multiShotButton.whenPressed(multiShotCommand, true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    MultistepAutonomousBuilder multiStepAutonomousBuilder = new MultistepAutonomousBuilder(driveSubsystem,
        navigationSubsystem, indexerSubsystem, shooterSubsystem, autoChooser.getSelected());
    return multiStepAutonomousBuilder.build();
  }

  private void setShuffleboardVals() {
    SmartDashboard.putBoolean("Shooter", false);
    SmartDashboard.putNumber("Speed", driveManuallyCommand.speed);
    SmartDashboard.putNumber("Rotation", driveManuallyCommand.rotation);
    SmartDashboard.putData("Power Distribution Panel", powerDistributionPanel);
    SmartDashboard.putData("Autonomous Chooser", autoChooser);
  }

  private void addSendableChooserOptions() {
    autoChooser.addOption("Left Shoot", 0);
    autoChooser.addOption("Middle Shoot", 1);
    autoChooser.addOption("Right Shoot", 2);
    autoChooser.addOption("Right Retreat", 3);
  }
}
