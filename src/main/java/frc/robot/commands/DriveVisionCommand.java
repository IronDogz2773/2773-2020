package frc.robot.commands;

//import frc.robot.Constants;
//import frc.robot.RobotContainer;
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
        // if(Math.abs(joy.getY()) > .15)
        // {
        //     speed = -joy.getY() * sAcc;
        //     if(sAcc < 1.00)
        //         sAcc += Constants.sInc;
        // }
        // else
        // {
        //     sAcc = 0;
        // }
        double alpha = angleEntry.getDouble(0);
        if(Math.abs(alpha) >= 3 && last != alpha)
        {
            rot = .25 * alpha / 10;
        }
        else
        {
            rot = 0;
        }
        last = alpha;
        driveSubsystem.manDrive(speed, rot);
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