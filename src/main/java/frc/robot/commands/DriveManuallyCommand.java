package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveManuallyCommand extends CommandBase {

     private final DriveSubsystem driveSubsystem;
     private double speed;
     private double rot;

    public DriveManuallyCommand(DriveSubsystem subsystem){
        driveSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        if(Math.abs(RobotContainer.joy.getY()) > .15)
            speed = -RobotContainer.joy.getY();
        if(Math.abs(RobotContainer.joy.getZ()) > .15)
            rot = RobotContainer.joy.getZ();
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