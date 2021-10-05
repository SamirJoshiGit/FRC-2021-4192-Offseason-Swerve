// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Auton.SampleAuton;
import frc.robot.commands.SwerveCommands.SwerveDriveControl;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.driveSystem.SwerveDrive;
import frc.robot.subsystems.driveSystem.SwerveModule;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.SwerveCommands.TestAngle;
import frc.robot.commands.SwerveCommands.TestPower;
import frc.robot.commands.SwerveCommands.TestPowerAngle;
import frc.robot.Constants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  //controllers
  private final XboxController driveControl = new XboxController(0);//Constants.driveControllerNum 
  
  //subsystems
  private final SwerveDrive swerve = new SwerveDrive();
  private final SwerveModule module = new SwerveModule(new TalonFX(14), new TalonFX(15), new CANCoder(0), Rotation2d.fromDegrees(0));
  private final SwerveModule module1 = new SwerveModule(new TalonFX(1), new TalonFX(0), new CANCoder(0), Rotation2d.fromDegrees(0));
  private final SwerveModule module2= new SwerveModule(new TalonFX(3), new TalonFX(2), new CANCoder(0), Rotation2d.fromDegrees(0));
  private final SwerveModule module3 = new SwerveModule(new TalonFX(12), new TalonFX(13), new CANCoder(0), Rotation2d.fromDegrees(0));
  
  //drive commands
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final SwerveDriveControl userControl = new SwerveDriveControl(swerve, driveControl);
  private final TestAngle turnTest = new TestAngle(module, .4);
  private final TestAngle turnTest1 = new TestAngle(module1, .4);
  private final TestAngle turnTest2 = new TestAngle(module2, .4);
  private final TestAngle turnTest3 = new TestAngle(module3, .4);
  private final TestPower powerTest = new TestPower(module, .4);
  private final TestPower powerTest1 = new TestPower(module1, .4);
  private final TestPower powerTest2 = new TestPower(module2, .4);
  private final TestPower powerTest3 = new TestPower(module3, .4);
  private final TestPowerAngle testBoth = new TestPowerAngle(module, .4, .4);

  //Buttons
  JoystickButton driverYButton = new JoystickButton(driveControl, 4);//set buttonNumbers in constants later
  JoystickButton driverXButton = new JoystickButton(driveControl, 3);//set buttonNumbers in constants later
  JoystickButton driverBButton = new JoystickButton(driveControl, 2);//set buttonNumbers in constants later
  //autonomous commands
  private final SampleAuton sampleAuton = new SampleAuton(swerve);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    //swerve.setDefaultCommand(userControl);
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverXButton.whenHeld(turnTest);
    driverXButton.whenHeld(turnTest1);
    driverXButton.whenHeld(turnTest2);
    driverXButton.whenHeld(turnTest3);
    driverYButton.whenHeld(powerTest);
    driverYButton.whenHeld(powerTest1);
    driverYButton.whenHeld(powerTest2);
    driverYButton.whenHeld(powerTest3);
    driverBButton.whenHeld(testBoth);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return sampleAuton;//temporary until we get auton commmand
  }
}
