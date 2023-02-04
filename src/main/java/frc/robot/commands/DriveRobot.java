// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
public class DriveRobot extends CommandBase {
  /** Creates a new DriveRobot. */
  public DriveRobot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_Drivetrain);
  }

  PIDController align = new PIDController(Constants.zpid.p,Constants.zpid.i,Constants.zpid.d);
  ADIS16470_IMU gyro = RobotContainer.m_imu;
  public static double vector(double x, double y) {
      double angleRadians = Math.atan2(y, x);
      double angleDegrees = Math.toDegrees(angleRadians);
      return angleDegrees;
  }
  private void moveAuto(double x, double y,double z,double angle) {
    RobotContainer.m_Drivetrain.driveCartesian(y,x,z,angle);

  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.m_imu.reset();
    RobotContainer.m_imu.calibrate();
    RobotContainer.m_imu.reset();
  }

  public boolean mode;
  private double angle;
  public double turbo;
  public double turboamount;
  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    
    turbo = RobotContainer.xbox.getRightTriggerAxis();
    
    if (turbo != 0) {
      turboamount = turbo+0.5;
    } else if (turbo >= 1) {
    turboamount = 1;
    }
    else {
      turboamount = Constants.c_speedcap;

    }
    
    SmartDashboard.putNumber("turbo amount", turboamount);
    SmartDashboard.putNumber("turbo", turbo);
    double joystickxz = RobotContainer.xbox.getLeftX(); // getRawAxis(Constants.c_leftJoystickAxisx);
    double joystickyz = RobotContainer.xbox.getLeftY()
    double joystickx = RobotContainer.xbox.getRightX(); // getRawAxis(Constants.c_rightJoystickAxisx);
    double joysticky = -RobotContainer.xbox.getRightY(); // getRawAxis(Constants.c_rightJoystickAxisy);
    double outputx = joystickx * turboamount;
    double outputy = joysticky * turboamount;
    double outputz = joystickxz * turboamount;
    mode = RobotContainer.DriveMode.getSelected();
    if (mode) {
      angle = -RobotContainer.m_imu.getAngle();
      
    } else {
      angle = 0;
    }
    if (RobotContainer.xbox.getLeftBumper()) {
      RobotContainer.m_imu.reset();
    }
    SmartDashboard.putNumber("x", outputx);
    SmartDashboard.putNumber("y", outputy);
    SmartDashboard.putNumber("z", outputz);
    SmartDashboard.putNumber("output heading", angle);
    SmartDashboard.putNumber("actual heading", -RobotContainer.m_imu.getAngle());;
    if (mode) {
      moveAuto(0,0,align.calculate(gyro.getAngle(),vector(joystickxz,joystickxz)),angle);    
    }
    else {
      moveAuto(outputx, outputy, outputz, angle);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
