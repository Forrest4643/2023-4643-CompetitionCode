// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.TurretPIDSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurretPosition extends PIDCommand {
  /** Creates a new TurretPosition. */
  public TurretPosition(TurretPIDSubsystem m_turretPIDsubsystem, double position) {
    super(
        // The controller that the command will use
        new PIDController(TurretConstants.turretkP, TurretConstants.turretkI, TurretConstants.turretkD),
        // This should return the measurement
        () -> m_turretPIDsubsystem.turretPositionDEG(),
        // This should return the setpoint (can also be a constant)
        () -> { 
          System.out.println("TurretPos Setpoint: " + position);
          return position; 
        },
        // This uses the output
        output -> {
          m_turretPIDsubsystem.setMotor(output);
        });

  }

  public double getSetpoint() {
    return this.m_controller.getSetpoint();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
