package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class Auto extends CommandBase{
   public Auto() {
    addRequirements(RobotContainer.m_Drivetrain);
   } 


Timer autoTime = new Timer();
public Boolean mode = true;

@Override
public void initialize(){
System.out.println(" stop being haunted");
    autoTime.reset();
    autoTime.stop();
}

@Override
public void execute() {
    autoTime.start();
    SmartDashboard.putNumber("autoTime",autoTime.get());
            RobotContainer.m_Drivetrain.driveCartesian(-0.5, 0, 0, 0);
    }     
  


@Override
public boolean isFinished(){
    return false;
}
}
