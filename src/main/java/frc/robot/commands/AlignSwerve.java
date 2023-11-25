// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import static frc.robot.Constants.DriveConstants.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignSwerve extends ParallelCommandGroup
{
  public AlignSwerve(DriveSubsystem swerve)
  {
    addCommands(
      new AlignModule(swerve.frontLeft, 0, alignP[0], alignI[0], alignD[0]),
      new AlignModule(swerve.frontRight, 0, alignP[1], alignI[1], alignD[1]),
      new AlignModule(swerve.backLeft, 0, alignP[2], alignI[2], alignD[2]),
      new AlignModule(swerve.backRight, 0, alignP[3], alignI[3], alignD[3])
    );
  }
}
