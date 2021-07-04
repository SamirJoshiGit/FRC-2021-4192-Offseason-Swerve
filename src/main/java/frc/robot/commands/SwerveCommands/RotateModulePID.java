// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.rotateTheModulePID;
import frc.robot.subsystems.driveSystem.SwerveModule;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateModulePID extends PIDCommand {
  /** Creates a new RotateModulePID. */
  public RotateModulePID(SwerveModule module, double xDir, double yDir) {
    super(
        // The controller that the command will use
        new PIDController(rotateTheModulePID.kRotateP, rotateTheModulePID.kRotateI, rotateTheModulePID.kRotateD),
        // This should return the measurement
        () -> module.getFromInit(),
        // This should return the setpoint (can also be a constant)
        () -> Math.atan(xDir/yDir),
        // This uses the output
        output -> {
          // Use the output here
          //SwerveModule.changeAngle(output);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
