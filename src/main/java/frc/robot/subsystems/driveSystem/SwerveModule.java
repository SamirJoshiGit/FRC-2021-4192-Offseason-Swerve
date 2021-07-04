// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.driveSystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swervePID;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private TalonFX driveMotor;
  private TalonFX angleMotor;
  private static CANCoder angleCoder;
  private static Encoder driveCoder;
  private static double initAngle;
  public SwerveModule(TalonFX driveMotor, TalonFX angleMotor, CANCoder canCoder, Rotation2d offset) {
    this.driveMotor = driveMotor;
    this.angleMotor = angleMotor;
    canCoder = angleCoder;
    initAngle = angleCoder.getPosition();

    TalonFXConfiguration angleTalonFXConfiguration = new TalonFXConfiguration();

    angleTalonFXConfiguration.slot0.kP = swervePID.kAngleP;
    angleTalonFXConfiguration.slot0.kI = swervePID.kAngleI;
    angleTalonFXConfiguration.slot0.kD = swervePID.kAngleD;

    angleTalonFXConfiguration.remoteFilter0.remoteSensorDeviceID = canCoder.getDeviceID();
    angleTalonFXConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    angleTalonFXConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
    angleMotor.configAllSettings(angleTalonFXConfiguration);

    TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();

    driveTalonFXConfiguration.slot0.kP = swervePID.kDriveP;
    driveTalonFXConfiguration.slot0.kI = swervePID.kDriveI;
    driveTalonFXConfiguration.slot0.kD = swervePID.kDriveD;
    driveTalonFXConfiguration.slot0.kF = swervePID.kDriveF;

    driveMotor.configAllSettings(driveTalonFXConfiguration);

    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.magnetOffsetDegrees = offset.getDegrees();
    canCoder.configAllSettings(canCoderConfiguration);

  }

  //change to non-static if not working and add gyro into drive also
  public static Rotation2d getRotationAngle(){
    return Rotation2d.fromDegrees(angleCoder.getAbsolutePosition());
  }
  public static double getFromInit(){
    return angleCoder.getPosition() - initAngle;
  }
  public SwerveModuleState getState() {
    return new SwerveModuleState(driveCoder.getRate(), new Rotation2d(angleCoder.getPosition()));
  }
  //parameter tells what state we want
  public void setDesiredState(SwerveModuleState desiredState){
    //gets our angle and sets optimal state
    Rotation2d currentRotation = getRotationAngle();
    SwerveModuleState state = optimize(desiredState, currentRotation);
    
    //finds the angle difference between current rotation and desired rotation
    Rotation2d rotationDelta = state.angle.minus(currentRotation);
    
    //finds ticks to desired position
    double deltaTicks = (rotationDelta.getDegrees() / 360) * swervePID.kEncoderTicksPerRotation;
    double currentTicks = angleCoder.getPosition() / angleCoder.configGetFeedbackCoefficient();
    
    //sets desired ticks to right position and sets it
    double desiredTicks = currentTicks + deltaTicks;
    angleMotor.set(TalonFXControlMode.Position, desiredTicks);

    //units to feet per second
    double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond);

    //send power
    driveMotor.set(TalonFXControlMode.PercentOutput, feetPerSecond / SwerveDrive.kMaxSpeed);
  }
  
  public void changeAngle(double output){
    angleMotor.set(ControlMode.PercentOutput, output);
  }
  
  public static SwerveModuleState optimize(
      SwerveModuleState desiredState, Rotation2d currentAngle) {
    var delta = desiredState.angle.minus(currentAngle);
    if (Math.abs(delta.getDegrees()) > 90.0) {
      return new SwerveModuleState(
          -desiredState.speedMetersPerSecond,
          desiredState.angle.rotateBy(Rotation2d.fromDegrees(180.0)));
    } else {
      return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
