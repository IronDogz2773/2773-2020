package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


public class DriveVisionCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;

    PIDController obamaController = new PIDController(0.05, 0, 0);
    private double speed;
    private double rot;
    private double last;
    private double target;
    private double error;

    //private double sAcc;
    //private double rAcc;

    private NetworkTableEntry angleEntry;

    public DriveVisionCommand(DriveSubsystem subsystem, Joystick joy){
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
    }

    @Override
    public void execute(){ //what the code does while the command is active

        double alpha = angleEntry.getDouble(0);
        if(alpha != last)
        {
            target = driveSubsystem.gyroscope.getAngle() - alpha;
            obamaController.setSetpoint(0);
            SmartDashboard.putNumber("Target", target);
        }
        error = target - driveSubsystem.gyroscope.getAngle();
        SmartDashboard.putNumber("Error", error);
        

        //if(!obamaController.atSetpoint())
        //{
          rot = obamaController.calculate(error);
        //}
        
        driveSubsystem.rawDrive(speed, rot);
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