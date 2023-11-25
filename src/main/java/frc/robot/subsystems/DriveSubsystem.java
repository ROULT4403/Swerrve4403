// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.DriveConstants.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase
{
  //CHANGED MODULES ORDER TO MAKE CODE CONSISTENT
  public final SwerveModule frontLeft =
      new SwerveModule(
          driveMotorIDs[0], turningMotorIDs[0], absoluteEncoderChannels[0],
          driveInverted[0], turningInverted[0], offset[0],
          turningP[0], turningI[0], turningD[0]);

  public final SwerveModule frontRight =
      new SwerveModule(
          driveMotorIDs[1], turningMotorIDs[1], absoluteEncoderChannels[1],
          driveInverted[1], turningInverted[1], offset[1],
          turningP[1], turningI[1], turningD[1]);

  public final SwerveModule backLeft =
      new SwerveModule(
          driveMotorIDs[2], turningMotorIDs[2], absoluteEncoderChannels[2],
          driveInverted[2], turningInverted[2], offset[2],
          turningP[2], turningI[2], turningD[2]);

  public final SwerveModule backRight =
      new SwerveModule(
          driveMotorIDs[3], turningMotorIDs[3], absoluteEncoderChannels[3],
          driveInverted[3], turningInverted[3], offset[3],
          turningP[3], turningI[3], turningD[3]);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  SwerveDriveOdometry odometry =
      new SwerveDriveOdometry(
          kinematics,
          gyro.getRotation2d(),
          new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
          });

  public DriveSubsystem() {}

  @Override
  public void periodic()
  {
    odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });
  
  SmartDashboard.putNumber("NavX", getHeading());
  }


  public Pose2d getPose()
  {
    return odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose)
  {
    odometry.resetPosition(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        },
        pose);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative)
  {
    var swerveModuleStates =
        kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, maxSpeedMPS);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates)
  {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, maxSpeedMPS);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders()
  {
    frontLeft.resetEncoders();
    backLeft.resetEncoders();
    frontRight.resetEncoders();
    backRight.resetEncoders();
  }

  public void zeroHeading()
  {
    gyro.reset();
  }

  public double getHeading()
  {
    return gyro.getRotation2d().getRadians();
  }

  /*public double getTurnRate()
  {
    return gyro.getRate() * (navXInverted ? -1.0 : 1.0);
  }*/
}