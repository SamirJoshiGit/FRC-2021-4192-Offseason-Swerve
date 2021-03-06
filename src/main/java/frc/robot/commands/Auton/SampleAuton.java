// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveCommands.DriveForSetTime;
import frc.robot.subsystems.driveSystem.SwerveDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SampleAuton extends SequentialCommandGroup {
  /** Creates a new SampleAuton. */
  public SampleAuton(SwerveDrive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveForSetTime(drive, 3, .33 , 0, 0, true), 
      new DriveForSetTime(drive, 3, 0, .33, 0, true),  
      new DriveForSetTime(drive, 3, .33, .33, 45, true));
  }
}
