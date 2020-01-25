package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;

public class DriveManuallyCommand extends CommandBase {

     private final DriveSubsystem driveSubsystem;
     private double speed;
     private double rotation;
     private Joystick joystick;

     private double speedAcceleration;
     private double rotationAcceleration;

    public DriveManuallyCommand(DriveSubsystem subsystem, Joystick joystick){
        driveSubsystem = subsystem;
        addRequirements(subsystem);
        this.joystick = joystick;
    }

    @Override
    public void initialize(){
        //Called at the beginning of each time command is used
        speedAcceleration = .6;
        rotationAcceleration = 1;
    }

    @Override
    public void execute(){ //what the code does while the command is active
        if(Math.abs(joystick.getY()) > .15)
        {
            speed = -joystick.getY() * speedAcceleration * Constants.movementSpeedCap;
            if(speedAcceleration < 1.00)
                speedAcceleration += Constants.speedIncrement;
        }
        else
        {
            speedAcceleration = 0.7;
        }
        if(Math.abs(joystick.getZ()) > .15)
        {
            rotation = joystick.getZ() * rotationAcceleration * Constants.rotationSpeedCap;
            if(rotationAcceleration < 1.00)
                rotationAcceleration += Constants.rotationIncrement;
        }
        else
        {
            rotationAcceleration = 0.5;
        }
        driveSubsystem.manualDrive(speed, rotation);
    }

    @Override
    public boolean isFinished(){
        return false;
    }

    @Override
    public void end(boolean interrupted){

    }
    
}