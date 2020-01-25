package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;

public class DriveManuallyCommand extends CommandBase {

     private final DriveSubsystem driveSubsystem;
     private double speed;
     private double change;
     private double rot;
     private Joystick joy;

     private double sAcc;
     private double rAcc;

    public DriveManuallyCommand(DriveSubsystem subsystem, Joystick joy){
        driveSubsystem = subsystem;
        addRequirements(subsystem);
        this.joy = joy;
    }

    @Override
    public void initialize(){
        //Called at the beginning of each time command is used
        sAcc = 0;
        rAcc = 0;
    }

    @Override
    public void execute(){ //what the code does while the command is active
        if(!driveSubsystem.driveState)
        {
            if(Math.abs(joy.getY()) > .15)
            {
                speed = -joy.getY() * sAcc;
                if(sAcc < 1.00)
                    sAcc += Constants.sInc;
            }
            else
            {
                sAcc = 0;
            }
            if(Math.abs(joy.getZ()) > .15)
            {
                rot = joy.getZ() * rAcc;
                if(rAcc < 1.00)
                    rAcc += Constants.rInc;
            }
            else
            {
                rAcc = 0;
            }
            driveSubsystem.manDrive(speed, rot);
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){

    }
    
}