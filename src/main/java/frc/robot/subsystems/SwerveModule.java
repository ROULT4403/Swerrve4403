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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

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

  private boolean absoluteEncoderReversed;

  private final PIDController turningPID;


  public SwerveModule(
      int driveMotorID,
      int turningMotorID,
      int absoluteEncoderChannel,
      boolean driveMotorReversed,
      boolean turningMotorReversed,
      double offsetRad,
      double turningP,
      double turningI, 
      double turningD)
      {
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        driveMotor.restoreFactoryDefaults();

        turningMotor = new TalonFX(turningMotorID);

        driveEncoder = driveMotor.getEncoder();
        absoluteEncoder = new DutyCycleEncoder(absoluteEncoderChannel);

        driveEncoder.setPositionConversionFactor(kDriveEncoderDistancePerPulse);
        driveEncoder.setVelocityConversionFactor(kDriveEncoderDistancePerPulse / 60);
        absoluteEncoder.setDistancePerRotation(2 * Math.PI);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveMotor.setIdleMode(IdleMode.kCoast);
        turningMotor.setNeutralMode(NeutralMode.Brake);

        this.absoluteEncoderReversed = turningMotorReversed;

        absoluteEncoder.setPositionOffset(offsetRad);

        turningPID = new PIDController(turningP, turningI, turningD);

        turningPID.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
      }


  /*public double getAngle()
  {
    if(absoluteEncoderReversed == true)
    {
      return -((turningMotor.getSelectedSensorPosition() / 16384) * 2 * Math.PI);
    }
    else
    {
      return ((turningMotor.getSelectedSensorPosition() / 16384) * 2 * Math.PI);
    }
  }*/

  public double getAngle()
  {
    return ((turningMotor.getSelectedSensorPosition() / 16384) * 2 * Math.PI);
  }

  public void alignModule(double input)
  {
    turningMotor.set(ControlMode.PercentOutput, input);
  }

  public SwerveModuleState getState()
  {
    return new SwerveModuleState(
        driveEncoder.getVelocity(), new Rotation2d(getAngle()));
  }

  public SwerveModulePosition getPosition()
  {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), new Rotation2d(getAngle()));
  }

  public double getAbsoluteAngle()
  {
    if(absoluteEncoderReversed == true)
    {
      return (absoluteEncoder.getAbsolutePosition());
    }
    else
    {
      return (absoluteEncoder.getAbsolutePosition());
    }
  }

  public void setDesiredState(SwerveModuleState desiredState)
  {
    SwerveModuleState state =
        SwerveModuleState.optimize(desiredState, new Rotation2d(getAngle()));

    final double driveOutput =
        state.speedMetersPerSecond;

    final double turnOutput =
        turningPID.calculate(getAngle(), state.angle.getRadians());

    driveMotor.set(driveOutput / maxSpeedMPS);
    turningMotor.set(ControlMode.PercentOutput, turnOutput);
  }

  public void resetEncoders()
  {
    driveEncoder.setPosition(0);
    turningMotor.setSelectedSensorPosition(0);
    //turningMotor.setSelectedSensorPosition(getAbsoluteAngle());
  }

  @Override
  public void periodic()
  {
    SmartDashboard.putNumber("Drive Velocity: " + driveMotor.getDeviceId(), getState().speedMetersPerSecond);
    SmartDashboard.putNumber("Drive Position: " + driveMotor.getDeviceId(), driveEncoder.getPosition());
    SmartDashboard.putNumber("Turning Angle: " + turningMotor.getDeviceID(), getState().angle.getRadians());
    SmartDashboard.putNumber("Absolute Angle: " + absoluteEncoder.getSourceChannel(), getAbsoluteAngle());

  }
}