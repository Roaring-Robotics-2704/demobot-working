// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;


public class Auto extends CommandBase {
  /** Creates a new Auto. */
  public Auto() {
    addRequirements(RobotContainer.m_Drivetrain);
  }
  Timer autoTime = new Timer();
  public Boolean mode = true;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoTime.reset();
    RobotContainer.m_imu.reset();
    autoTime.reset();
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    autoTime.start();
    while (autoTime.get() <= 2.4) {
      RobotContainer.m_Drivetrain.driveCartesian(-0.5, 0, 0,Rotation2d.fromRotations(RobotContainer.m_imu.getAngle()));
    }
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
