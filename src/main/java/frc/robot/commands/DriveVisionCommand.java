package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;

public class DriveVisionCommand extends CommandBase {

    private final DriveSubsystem driveSubsystem;

    private double speed;
    private double rot;
    private double last;
    //private Joystick joy;

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