// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.OIConstants.*;

import frc.robot.commands.AlignSwerve;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer
{
    private final DriveSubsystem swerve = new DriveSubsystem();
    //ADDED LIMITER AND DEADBAND TO AID CONTROL AND DEBUGGING
    CommandXboxController driver = new CommandXboxController(driverPort);
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(rateLimiter);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(rateLimiter);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(rateLimiter);

    private final AlignSwerve alignSwerve = new AlignSwerve(swerve); 

    Trigger align = driver.start();
    
    public RobotContainer()
    {
        alignSwerve.execute();
        configureButtonBindings();

        swerve.setDefaultCommand(
            new RunCommand(
                () ->
                    swerve.drive(
                        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(driver.getLeftY(), deadband)),
                        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(driver.getLeftX(), deadband)),
                        -m_rotLimiter.calculate(MathUtil.applyDeadband(driver.getRightX(), deadband)),
                        fieldOriented),
                swerve));
    }

    private void configureButtonBindings()
    {
        align.onTrue(alignSwerve);
        align.onFalse(null);
    }
    
    public Command getAutonomousCommand()
    {
        return null;
    }
}
