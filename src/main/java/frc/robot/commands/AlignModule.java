// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.SwerveModule;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignModule extends PIDCommand
{
  /** Creates a new AlignModule. */
  public AlignModule(SwerveModule module, double setpoint, double alignP, double alignI, double alignD)
  {
    super(
        // The controller that the command will use
        new PIDController(alignP, alignI, alignD),
        // This should return the measurement
        module::getAngle,
        // This should return the setpoint (can also be a constant)
        setpoint,
        // This uses the output
        output -> {
          // Use the output here
          module.alignModule(output);
        });
    getController().setTolerance(0.01);
    addRequirements(module);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return getController().atSetpoint();
  }
}
