// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.TurretPIDSubsystem;

public class TrackTarget extends CommandBase {
  private final TurretPIDSubsystem m_turretPIDsubsystem;
  /** Creates a new TrackTarget. */
  public TrackTarget(TurretPIDSubsystem m_turretPIDsubsystem) {
    this.m_turretPIDsubsystem = m_turretPIDsubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turretPIDsubsystem.enable();
    m_turretPIDsubsystem.setSetpoint(6);
    System.out.println("TrackTarget Started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("turretPositionDEG", m_turretPIDsubsystem.turretPositionDEG());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turretPIDsubsystem.disable();
    System.out.println("TrackTarget Ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
