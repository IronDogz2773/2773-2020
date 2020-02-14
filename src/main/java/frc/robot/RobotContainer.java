/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CompressorControlCommand;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.DriveVisionCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeSpinCommand;
import frc.robot.commands.ResetGyroscopeCommand;
import frc.robot.commands.StartSpinCommand;
import frc.robot.commands.TurnDegreesCommand;
import frc.robot.subsystems.AirSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
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

  // Subsystems
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final NavigationSubsystem navigationSubsystem = new NavigationSubsystem();
  private final AirSubsystem airSubsystem = new AirSubsystem();
  private final IndexerSubsystem indexerSubsystem = new IndexerSubsystem();

  // Commands
  private final DriveManuallyCommand driveManuallyCommand = new DriveManuallyCommand(driveSubsystem, joystick);
  private final StartSpinCommand startSpinCommand = new StartSpinCommand(shooterSubsystem);
  private final IntakeSpinCommand intakeSpinCommand = new IntakeSpinCommand(intakeSubsystem, gamepad);
  private final DriveVisionCommand visionCommand = new DriveVisionCommand(driveSubsystem, navigationSubsystem);
  private final ResetGyroscopeCommand resetGyroscopeCommand = new ResetGyroscopeCommand(navigationSubsystem);
  private final CompressorControlCommand compressorControlCommand = new CompressorControlCommand(airSubsystem);
  private final IndexerCommand indexerCommand = new IndexerCommand(indexerSubsystem, gamepad);
  private final TurnDegreesCommand turn90Command = new TurnDegreesCommand(driveSubsystem, navigationSubsystem, 90);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    setShuffleboardVals();
    driveSubsystem.setDefaultCommand(driveManuallyCommand);
    //intakeSubsystem.setDefaultCommand(intakeSpinCommand);
    //airSubsystem.setDefaultCommand(compressorControlCommand);
    indexerSubsystem.setDefaultCommand(indexerCommand);

    // shooterSubsystem.setDefaultCommand(spinCmd);
    // TODO give remaining subsystems default commands
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
    // An ExampleCommand will run in autonomous
    return visionCommand;
  }

  private void setShuffleboardVals() {
    SmartDashboard.putBoolean("Shooter", false);
    SmartDashboard.putNumber("Speed", driveManuallyCommand.speed);
    SmartDashboard.putNumber("Rotation", driveManuallyCommand.rotation);
    SmartDashboard.putData("Power Distribution Panel", powerDistributionPanel);
  }
}
