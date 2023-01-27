// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class DriveRobot extends CommandBase {
  /** Creates a new DriveRobot. */
  public DriveRobot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Drivetrain);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
	public static final ADIS16470_IMU imu = new ADIS16470_IMU();


  // Called every time the scheduler runs while the command is scheduled.
  
  @Override
  public void execute() {
    double joystickLeftx = RobotContainer.xbox.getLeftX();   //getRawAxis(Constants.c_leftJoystickAxisx);
    double joystickRightx = RobotContainer.xbox.getRightX();  //getRawAxis(Constants.c_rightJoystickAxisx);
    double joystickRighty = -RobotContainer.xbox.getRightY(); //getRawAxis(Constants.c_rightJoystickAxisy);
    double outputx = joystickRightx*Constants.c_speedcap;
    double outputy = joystickRighty*Constants.c_speedcap;
    double outputz = joystickLeftx*Constants.c_speedcap;
    double angle = imu.getAngle();
    Rotation2d rotation = Rotation2d.fromRadians(angle);

    RobotContainer.m_Drivetrain.driveCartesian(outputy,outputx,outputz,rotation);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
