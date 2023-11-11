// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.OIConstants.*;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    private final DriveSubsystem swerve = new DriveSubsystem();
    //ADDED LIMITER AND DEADBAND TO AID CONTROL AND DEBUGGING
    CommandXboxController driver = new CommandXboxController(driverPort);
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
    
    public RobotContainer()
   
    {
        configureButtonBindings();

        swerve.setDefaultCommand(
            new RunCommand(
                () ->
                    swerve.drive(
                        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(driver.getLeftY(), 0.25)),
                        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(driver.getLeftX(), 0.25)),
                        -m_rotLimiter.calculate(MathUtil.applyDeadband(driver.getRightX(), 0.25)),
                       fieldOriented),
                swerve));
    }

    
    private void configureButtonBindings()
    {
        driver.a().onTrue(new InstantCommand(() -> swerve.startModules()));
    }
    
    public Command getAutonomousCommand()
    {
        return null;
    }
}
