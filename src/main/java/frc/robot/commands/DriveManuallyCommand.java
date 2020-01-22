package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;

public class DriveManuallyCommand extends CommandBase {

     private final DriveSubsystem driveSubsystem;
     private double speed;
     private double change;
     private double rot;
     private Joystick joy;

    public DriveManuallyCommand(DriveSubsystem subsystem, Joystick joy){
        driveSubsystem = subsystem;
        addRequirements(subsystem);
        this.joy = joy;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(Math.abs(joy.getY()) > .15){
            change = joy.getY() - speed;
            speed += change * 0.1;
        }
        else{
            speed = 0;
        }
        if(Math.abs(joy.getZ()) > .15)
            rot = joy.getZ();
        driveSubsystem.manDrive(speed, rot);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){

    }
    
}