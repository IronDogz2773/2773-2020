package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveManuallyCommand extends CommandBase {

     private final DriveSubsystem driveSubsystem;

    public DriveManuallyCommand(DriveSubsystem subsystem){
        driveSubsystem = subsystem;
        addRequirements(subsystem); //have to have this
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        double speed = -RobotContainer.joy.getY();
        double rot = RobotContainer.joy.getZ();
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