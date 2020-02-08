package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.NavigationSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;

public class DriveVisionCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    PIDController obamaController = new PIDController(0.03, 0.02, 0);
    private double rot;
    private double target;
    private final NavigationSubsystem nav;

    public DriveVisionCommand(DriveSubsystem subsystem, NavigationSubsystem nav){
        driveSubsystem = subsystem;
        addRequirements(subsystem);

        this.nav = nav;
    }

    @Override
    public void initialize(){
        driveSubsystem.driveState = true;
        obamaController.setTolerance(1);
        obamaController.setSetpoint(0);

        target = 10;
        obamaController.setSetpoint(target);
    }

    @Override
    public void execute(){ //what the code does while the command is active
        double alpha = nav.getVisionAngle();  
        SmartDashboard.putNumber("Target", target);
        double e = obamaController.calculate(nav.getGyroAngle()); 
        rot = MathUtil.clamp(e, -0.55, 0.55);
        
        driveSubsystem.rawDrive(0, rot, false);
        driveSubsystem.driveState = true;
        SmartDashboard.putNumber("Alpha", alpha);
        SmartDashboard.putNumber("Rotation", rot);
        

        
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){
        driveSubsystem.driveState = false;
    }
    
}