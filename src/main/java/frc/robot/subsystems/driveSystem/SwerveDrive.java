// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.driveSystem;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;

public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  public static final double kMaxSpeed = Units.feetToMeters(13.6);

  private AHRS ahrs = new AHRS(SPI.Port.kMXP);


  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    //change when building drive
    new Translation2d(
      Units.inchesToMeters(7.5),
      Units.inchesToMeters(7.5)
    ),
    new Translation2d(
      Units.inchesToMeters(7.5),
      Units.inchesToMeters(-7.5)
    ),
    new Translation2d(
      Units.inchesToMeters(-7.5),
      Units.inchesToMeters(7.5)
    ),
    new Translation2d(
      Units.inchesToMeters(-7.5),
      Units.inchesToMeters(-7.5)
    )
  );
  //probe rotation angle
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics,
  SwerveModule.getRotationAngle(), new Pose2d(0, 0, new Rotation2d()));
  
  //CAN numbers need to be changed to match PDP CAN
  private SwerveModule[] modules = new SwerveModule[] {
    new SwerveModule(new TalonFX(1), new TalonFX(2), new CANCoder(0), Rotation2d.fromDegrees(0)), // Front Left
    new SwerveModule(new TalonFX(3), new TalonFX(4), new CANCoder(1), Rotation2d.fromDegrees(0)), // Front Right
    new SwerveModule(new TalonFX(5), new TalonFX(6), new CANCoder(2), Rotation2d.fromDegrees(0)), // Back Left
    new SwerveModule(new TalonFX(7), new TalonFX(8), new CANCoder(3), Rotation2d.fromDegrees(0))  // Back Right
  };
  
  public SwerveDrive() {
    ahrs.reset();

  }

  public double getAngle(){
    return ahrs.getAngle();
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] states =
      kinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, ahrs.getRotation2d())
          : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(states, kMaxSpeed);
    for (int i = 0; i < states.length; i++) {
      SwerveModule module = modules[i];
      SwerveModuleState state = states[i];
      module.setDesiredState(state);
    }
  }
  //updates pose
  public void updatePose(){
    var gyroAngle = Rotation2d.fromDegrees(-getAngle());
    odometry.update(ahrs.getRotation2d(), modules[0].getState(), modules[1].getState(), modules[2].getState(), modules[3].getState());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updatePose();
  }
}
