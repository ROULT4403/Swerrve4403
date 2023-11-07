// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.OIConstants.*;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    private final DriveSubsystem swerve = new DriveSubsystem();

    CommandXboxController driver = new CommandXboxController(driverPort);

    Trigger start = driver.start();
    Trigger stop = driver.back();
    
    public RobotContainer()
    {
        configureButtonBindings();

        swerve.setDefaultCommand(
            new RunCommand(
                () ->
                    swerve.drive(
                        -driver.getLeftY(),
                        -driver.getLeftX(),
                        driver.getRightX(),
                        fieldOriented),
                swerve));
    }

    
    private void configureButtonBindings()
    {
        start.onTrue(new RunCommand(() -> swerve.startModules(), swerve));
        start.onFalse(null);

        stop.onTrue(new RunCommand(() -> swerve.stopModules(), swerve));
        stop.onFalse(null);
    }
    
    public Command getAutonomousCommand()
    {
        return null;
    }
}
