package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Auto extends CommandBase{
   public Auto() {
    addRequirements(RobotContainer.m_Drivetrain);
   } 
private void moveAuto(double y,double x,double z) {
RobotContainer.m_Drivetrain.driveCartesian(y,x,z,0);
    
}

Timer autoTime = new Timer();
public int mode = 1;

@Override
public void initialize(){
    autoTime.reset();
}

@Override
public void execute() {
    autoTime.start();
    SmartDashboard.putNumber("autoTime",autoTime.get());
    if (mode == 1) {
        while (autoTime.get() <= 1){//backwards
            moveAuto(-0.3,0,0);
        }
        
        while ( autoTime.get() <= 2){//right
            moveAuto(0,0.3,0);
        }
        
        while  ( autoTime.get() <= 3){//forwards
            moveAuto(0.3,0,0);
        }
        while  ( autoTime.get() <= 4){//left
            moveAuto(0,-0.3,0);
        }
    }     
     
    else if (mode == 2) {
        while ( autoTime.get() <= 1.6){
            moveAuto(-0.3,0,0);
            
        
        }
    }
    else if (mode == 3) {
        while (autoTime.get()<= 5.5){
            moveAuto(0.5,0,0);
        }
        while (autoTime.get()<=10){
            moveAuto(0,-0.5,0);
        }
        while(autoTime.get()<= 15){
            moveAuto(-0.5,0,0);
        }
    }
}

@Override
public boolean isFinished(){
    return false;

    
}}

