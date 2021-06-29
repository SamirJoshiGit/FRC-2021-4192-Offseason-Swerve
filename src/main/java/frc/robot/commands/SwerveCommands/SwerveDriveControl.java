// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveSystem.SwerveDrive;
import edu.wpi.first.wpilibj.GenericHID;

public class SwerveDriveControl extends CommandBase {
  /** Creates a new SwerveDriveControl. */
  private final SwerveDrive drivetrain;
  private final XboxController controller;

  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  public SwerveDriveControl(SwerveDrive drivetrain, XboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double speedInXDir = -xspeedLimiter.calculate(controller.getY(GenericHID.Hand.kLeft)) * drivetrain.kMaxSpeed;
    final double speedInYDir = -yspeedLimiter.calculate(controller.getX(GenericHID.Hand.kLeft)) * drivetrain.kMaxSpeed;
    final double speedRot = -rotLimiter.calculate(controller.getX(GenericHID.Hand.kRight)) * drivetrain.kMaxSpeed;
    boolean fieldRelative = controller.getBumper(GenericHID.Hand.kLeft);

    drivetrain.drive(speedInXDir, speedInYDir, speedRot, fieldRelative);

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
