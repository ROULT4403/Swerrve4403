// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import  static frc.robot.Constants.ModuleConstants.*;

public class SwerveModule  extends SubsystemBase
{
  private final CANSparkMax driveMotor;
  private final TalonFX turningMotor;

  private final RelativeEncoder driveEncoder;
  private final DutyCycleEncoder absoluteEncoder;

  private final PIDController drivePID;

  private final ProfiledPIDController turningPID;

  private final double offset;


  public SwerveModule(
      int driveMotorID,
      int turningMotorID,
      int absoluteEncoderChannel,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      double offsetRad,
      double driveP, double driveI,
      double turnP, double turnI)
      {
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();

        turningMotor = new TalonFX(turningMotorID);

        driveEncoder = driveMotor.getEncoder();
        absoluteEncoder = new DutyCycleEncoder(absoluteEncoderChannel);

        driveEncoder.setPositionConversionFactor(kDriveEncoderDistancePerPulse);
        driveEncoder.setVelocityConversionFactor(kDriveEncoderDistancePerPulse / 60);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveMotor.setIdleMode(IdleMode.kCoast);
        turningMotor.setNeutralMode(NeutralMode.Brake);

        this.offset = offsetRad;

        drivePID = new PIDController(driveP, driveI, 0);
        turningPID = new ProfiledPIDController(turnP, turnI, 0, new TrapezoidProfile.Constraints(turningSpeed, turningAcceleration));

        turningPID.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
      }


  private double getAngle()
  {
    return ((turningMotor.getSelectedSensorPosition() / 16384) * 2 * Math.PI);
  }

  public SwerveModuleState getState()
  {
    return new SwerveModuleState(
        driveEncoder.getVelocity(), new Rotation2d(getAngle() - offset));
  }

  public SwerveModulePosition getPosition()
  {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), new Rotation2d(getAngle() - offset));
  }

  public double getAbsoluteAngle()
  {
    return (absoluteEncoder.getAbsolutePosition() * 360) - offset;
  }

  public void setDesiredState(SwerveModuleState desiredState)
  {
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getAngle()));

    final double driveOutput =
        drivePID.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond);

    final double turnOutput =
        turningPID.calculate(getAngle(), state.angle.getRadians());

    driveMotor.set(driveOutput);
    turningMotor.set(ControlMode.PercentOutput, turnOutput);
  }

  public void resetEncoders()
  {
    driveEncoder.setPosition(0);
    turningMotor.setSelectedSensorPosition(getAbsoluteAngle());
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Drive Velocity: " + driveMotor.getDeviceId(), getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Drive Velocity: " + driveMotor.getDeviceId(), driveEncoder.getPosition());
    SmartDashboard.putNumber("Turning Angle: " + turningMotor.getDeviceID(), getState().angle.getRadians());
    SmartDashboard.putNumber("Absolute Angle: " + absoluteEncoder.getSourceChannel(), getAbsoluteAngle());
  }
}