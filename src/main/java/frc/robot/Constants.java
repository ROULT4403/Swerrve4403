// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
  public static final class DriveConstants
  {
    //CORRECTED IDS TO MATCH THE CORRECT ORDER, COMMENTS WITH PREVIOUS CONFIG
    public static final int[] driveMotorIDs =           new int[] {10, 1, 3, 4}; //{10, 3, 1, 4};
    public static final int[] turningMotorIDs =         new int[] {5, 6, 7, 8}; //{5, 7, 6, 8}
    public static final int[] absoluteEncoderChannels = new int[] {5, 6, 7, 9}; //{5, 7, 6, 9}
    public static final boolean[] driveInverted = new boolean[] {true, true, false, true};
    public static final boolean[] turningInverted = new boolean[] {true, false, false, true};
    
    //NEED TO REVIEW THE CORRECT OFFSET FOR MODULES 
    public static final double[] offset = new double[] {0.89, 0.2, 0, 1.5}; //{0.89, 0.2, 0, 1.97};
    
    //NEED TO REVIEW WHY MODULE BACKRIGHT GOES CRAZY
    public static final double[] turningP = new double[] {0.25, 0.25, 0.25, 0.18};//0.035, 0.0355, 0.0355, 0.035};
    public static final double[] turningI = new double[] {0.0, 0.0, 0.0, 0.0};
    public static final double[] turningD = new double[] {0.0, 0.0, 0.0, 0.0};

    public static final double kTrackWidth = Units.inchesToMeters(19);

    public static final double kWheelBase = Units.inchesToMeters(19);
    //CHANGED TO MAKE CONSISTENT VS GYRO
    public static final SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, kTrackWidth / 2));

    public static final boolean navXInverted = false;

    public static final double maxSpeedMPS = 3;

    public static final double deadband = 1;
  }

  public static final class ModuleConstants
  {
    public static final double turningSpeed = 2 * Math.PI;
    public static final double turningAcceleration = 2 * Math.PI;

    public static final int kEncoderCPR = 42 * (27/4);
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double maxSpeedMPS = 3;
  }

  public static final class OIConstants
  {
    public static final int driverPort = 0;

    public static final boolean fieldOriented = false;
  }
}
