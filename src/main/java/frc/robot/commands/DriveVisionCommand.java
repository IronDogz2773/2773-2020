package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpiutil.math.MathUtil;

public class DriveVisionCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    PIDController obamaController = new PIDController(0.03, 0.02, 0);
    private double speed;
    private double rot;
    private double last;
    private double target;
    private NetworkTableEntry angleEntry;

    public DriveVisionCommand(DriveSubsystem subsystem){
        driveSubsystem = subsystem;
        addRequirements(subsystem);
        //this.joy = joy;
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        angleEntry = inst.getEntry("/angle");
    }

    @Override
    public void initialize(){
        //Called at the beginning of each time command is used
        //sAcc = 0;
        //rAcc = 0;
        driveSubsystem.driveState = true;
        obamaController.setTolerance(1);
        obamaController.setSetpoint(0);

        target = 10;
        obamaController.setSetpoint(target);
    }

    @Override
    public void execute(){ //what the code does while the command is active
        double alpha = angleEntry.getDouble(0);  
        SmartDashboard.putNumber("Target", target);
        double e = obamaController.calculate(driveSubsystem.gyroscope.getAngle()); 
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