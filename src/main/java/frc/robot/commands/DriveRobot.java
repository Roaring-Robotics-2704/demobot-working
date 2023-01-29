// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
    RobotContainer.m_imu.reset();
    RobotContainer.m_imu.calibrate();
    RobotContainer.m_imu.reset();
  }

  public boolean mode;
  private double angle;
  public boolean turbo;
  public double turboamount;
  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    turbo = RobotContainer.xbox.getRightBumper();
    if (turbo) {
      turboamount = 1;
    } else {
      turboamount = Constants.c_speedcap;
    }
    double joystickz = RobotContainer.xbox.getLeftX(); // getRawAxis(Constants.c_leftJoystickAxisx);
    double joystickx = RobotContainer.xbox.getRightX(); // getRawAxis(Constants.c_rightJoystickAxisx);
    double joysticky = -RobotContainer.xbox.getRightY(); // getRawAxis(Constants.c_rightJoystickAxisy);
    double outputx = joystickx * turboamount;
    double outputy = joysticky * turboamount;
    double outputz = joystickz * turboamount;
    mode = RobotContainer.DriveMode.getSelected();
    if (mode) {
      angle = -RobotContainer.m_imu.getAngle();
      if (RobotContainer.xbox.getLeftBumper()) {
        RobotContainer.m_imu.reset();
      }
    } else {
      angle = 0;
    }
    RobotContainer.m_Drivetrain.driveCartesian(outputy, outputx, outputz, angle);
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
