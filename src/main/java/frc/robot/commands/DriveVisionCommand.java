package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveVisionCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;
    
    PIDController obamaController = new PIDController(1.0, 1.0, 1.0);
    private double speed;
    private double rot;
    private double last;

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
    }

    @Override
    public void execute(){ //what the code does while the command is active

        double alpha = angleEntry.getDouble(0);

        if(!obamaController.atSetpoint())
        {
          rot = obamaController.calculate(alpha);
        }
        
        driveSubsystem.rawDrive(speed, rot);
        driveSubsystem.driveState = true;
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