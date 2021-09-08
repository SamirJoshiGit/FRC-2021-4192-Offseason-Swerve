// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.driveSystem.SwerveModule;

public class TestPower extends CommandBase {
  /** Creates a new TestPower. */
  private SwerveModule module;
  private double powerOutput;
  public TestPower(SwerveModule module, double powerOutput) {
    this.module = module;
    this.powerOutput = powerOutput;
    addRequirements(module);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    module.testPower(powerOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    module.testPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
