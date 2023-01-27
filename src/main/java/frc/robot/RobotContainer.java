// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Auto;
import frc.robot.commands.DriveRobot;
import frc.robot.subsystems.Drivetrain;



/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //Subsystems
  public static Drivetrain m_Drivetrain = new Drivetrain();


  //Commands
  public static DriveRobot m_DriveRobot = new DriveRobot();
  public static Auto m_autonomous = new Auto();


  //OI
  public static XboxController xbox = new XboxController(Constants.c_joystick);
  //getPOV can be used to find the ange value of the d-Pad on the xbox controller


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //Is nessary, might have been the reason for the error "DifferntialDrive...Output not updated often enough"
    m_Drivetrain.setDefaultCommand(m_DriveRobot);
 
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomous;
  }
}
