// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveSystem.SwerveDrive;

public class DriveForSetTime extends CommandBase {
  /** Creates a new DriveForSetTime. */
  private SwerveDrive drive;
  private Timer clock;
  private double forTime;
  private double yDir;
  private double xDir;
  private double rotation;
  private boolean fieldRelative;
  public DriveForSetTime(SwerveDrive drive, double forTime, double yDir, double xDir, double rotation, boolean fieldRelative){
    this.drive = drive;
    this.forTime = forTime;
    this.yDir = yDir;
    this.xDir = xDir;
    this.rotation = rotation;
    this.fieldRelative = fieldRelative;
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    clock.reset();
    clock.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.drive(xDir, yDir, rotation, fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0, 0, fieldRelative);
    clock.stop();
    clock.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return clock.hasElapsed(forTime);
  }
}
