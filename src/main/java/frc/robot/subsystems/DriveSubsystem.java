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

import static frc.robot.Constants.DriveConstants.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase
{
  private final SwerveModule frontLeft =
      new SwerveModule(
          driveMotorIDs[0], turningMotorIDs[0], absoluteEncodersChannels[0],
          driveInverted[0], turningInverted[0],
          offset[0], driveP[0], driveI[0], turnP[0], turnI[0]);

  private final SwerveModule backLeft =
      new SwerveModule(
          driveMotorIDs[1], turningMotorIDs[1], absoluteEncodersChannels[1],
          driveInverted[1], turningInverted[1],
          offset[1], driveP[1], driveI[1], turnP[1], turnI[1]);

  private final SwerveModule frontRight =
      new SwerveModule(
          driveMotorIDs[2], turningMotorIDs[2], absoluteEncodersChannels[2],
          driveInverted[2], turningInverted[2],
          offset[2], driveP[2], driveI[2], turnP[2], turnI[2]);

  private final SwerveModule backRight =
      new SwerveModule(
          driveMotorIDs[3], turningMotorIDs[3], absoluteEncodersChannels[3],
          driveInverted[3], turningInverted[3],
          offset[3], driveP[3], driveI[3], turnP[3], turnI[3]);

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
    return gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate()
  {
    return gyro.getRate() * (navXInverted ? -1.0 : 1.0);
  }
}