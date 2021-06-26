// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static double driveTrainWidth = 0;
    public static double driveTrainLength = 0;
    public static double inchesToMeters = .0254;
    
    public static class swervePID{
        public static double kDriveP = 15.0;
        public static double kDriveI = 0.01;
        public static double kDriveD = 0.1;
        public static double kDriveF = 0.2;
      
        public static double kAngleP = 1.0;
        public static double kAngleI = 0.0;
        public static double kAngleD = 0.0;

        public static double kEncoderTicksPerRotation = 2048;//4096

        public static final double kMaxAngularSpeed = Math.PI;
    }
}


