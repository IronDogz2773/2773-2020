/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DriveManuallyCommand;
import frc.robot.commands.DriveVisionCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.StartSpinCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Joystick joystick = new Joystick(Constants.joystickPort);

  //Subsystems
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  //Commands
  private final ExampleCommand m_autonomousCommand = new ExampleCommand(m_exampleSubsystem);
  private final DriveManuallyCommand driveManuallyCommand = new DriveManuallyCommand(driveSubsystem, joystick);
  private final StartSpinCommand spinCommand = new StartSpinCommand(shooterSubsystem, joystick);
  private final DriveVisionCommand visionCommand = new DriveVisionCommand(driveSubsystem, joy);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    driveSubsystem.setDefaultCommand(driveManuallyCommand);
    //shooterSubsystem.setDefaultCommand(spinCmd);
    // TODO give remaining subsystems default commands
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton spinButton = new JoystickButton(joystick, Constants.spinButton);
    spinButton.whenPressed(spinCommand);
    JoystickButton visionButton = new JoystickButton(joystick, Constants.spinButton);
    visionButton.whenHeld(visionCommand, true);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autonomousCommand;
  }
}
