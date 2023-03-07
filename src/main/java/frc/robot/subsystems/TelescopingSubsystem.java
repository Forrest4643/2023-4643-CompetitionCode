// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class TelescopingSubsystem extends ProfiledPIDSubsystem implements Loggable{
  private static double m_kP = 0;
  @Config
  public void setTelescopingkP(double kP) {
    if(kP != 0) {
      m_kP = kP;
    }
  }

  private static double m_kI = 0;
  @Config
  public void setTelescopingkI(double kI) {
    if(kI != 0) {
      m_kI = kI;
    }
  }

  private static double m_kD = 0;
  @Config
  public void setTelescopingkD(double kD) {
    if(kD != 0) {
      m_kD = kD;
    }
  }
  /** Creates a new TelescopingSubsystem. */
  public TelescopingSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            0,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(0, 0)));
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Use the output (and optionally the setpoint) here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
}
