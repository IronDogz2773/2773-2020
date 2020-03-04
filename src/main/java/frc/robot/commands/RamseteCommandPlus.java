/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Modified by 2773 team to add proximity sensor

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * A command that uses a RAMSETE controller ({@link RamseteController}) to
 * follow a trajectory {@link Trajectory} with a differential drive.
 *
 * <p>
 * The command handles trajectory-following, PID calculations, and feedforwards
 * internally. This is intended to be a more specific "complete solution" that
 * can be used by teams without a great deal of controls expertise.
 *
 * <p>
 * Advanced teams seeking more flexibility (for example, those who wish to use
 * the onboard PID functionality of a "smart" motor controller) may use the
 * secondary constructor that omits the PID and feedforward functionality,
 * returning only the raw wheel speeds from the RAMSETE controller.
 */
@SuppressWarnings("PMD.TooManyFields")
public class RamseteCommandPlus extends CommandBase {
    private final Timer m_timer = new Timer();
    private final boolean m_usePID;
    private final Trajectory m_trajectory;
    private final RamseteController m_follower;
    private final SimpleMotorFeedforward m_feedforward;
    private final DifferentialDriveKinematics m_kinematics;
    private final PIDController m_leftController;
    private final PIDController m_rightController;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;

    private DriveSubsystem driveSubsystem;
    private NavigationSubsystem navigationSubsystem;

    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided
     * trajectory. PID control and feedforward are handled internally, and outputs
     * are scaled -12 to 12 representing units of volts.
     *
     * <p>
     * Note: The controller will *not* set the outputVolts to zero upon completion
     * of the path - this is left to the user, since it is not appropriate for paths
     * with nonstationary endstates.
     *
     * @param trajectory      The trajectory to follow.
     * @param driveSubsystem
     * @param navigationSubsystem
     * @param requirementsTODO    The subsystems to require.
     */
    @SuppressWarnings("PMD.ExcessiveParameterList")
    public RamseteCommandPlus(Trajectory trajectory, DriveSubsystem driveSubsystem,
            NavigationSubsystem navigationSubsystem) {

        m_trajectory = requireNonNullParam(trajectory, "trajectory", "RamseteCommand");
        
        m_follower = new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta);
        m_feedforward = new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter,
                Constants.kaVoltSecondsSquaredPerMeter);
        m_kinematics = Constants.kDriveKinematics;
        m_leftController = new PIDController(Constants.kPDriveVel, 0, 0);
        m_rightController = new PIDController(Constants.kPDriveVel, 0, 0);

        m_usePID = true;

        this.driveSubsystem = driveSubsystem;
        this.navigationSubsystem = navigationSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_prevTime = 0;
        var initialState = m_trajectory.sample(0);
        m_prevSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(initialState.velocityMetersPerSecond, 0,
                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
        m_timer.reset();
        m_timer.start();
        if (m_usePID) {
            m_leftController.reset();
            m_rightController.reset();
        }
    }

    @Override
    public void execute() {
        double curTime = m_timer.get();
        double dt = curTime - m_prevTime;

        var pose = navigationSubsystem.getPose();
        var targetWheelSpeeds = m_kinematics.toWheelSpeeds(m_follower.calculate(pose, m_trajectory.sample(curTime)));

        var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftOutput;
        double rightOutput;

        if (m_usePID) {
            double leftFeedforward = m_feedforward.calculate(leftSpeedSetpoint,
                    (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

            double rightFeedforward = m_feedforward.calculate(rightSpeedSetpoint,
                    (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

            var speeds = navigationSubsystem.getWheelSpeeds();
            leftOutput = leftFeedforward + m_leftController.calculate(speeds.leftMetersPerSecond, leftSpeedSetpoint);

            rightOutput = rightFeedforward
                    + m_rightController.calculate(speeds.rightMetersPerSecond, rightSpeedSetpoint);
        } else {
            leftOutput = leftSpeedSetpoint;
            rightOutput = rightSpeedSetpoint;
        }

        driveSubsystem.tankDriveVolts(leftOutput, rightOutput);

        m_prevTime = curTime;
        m_prevSpeeds = targetWheelSpeeds;
    }

    @Override
    public void end(boolean interrupted) {
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
}
