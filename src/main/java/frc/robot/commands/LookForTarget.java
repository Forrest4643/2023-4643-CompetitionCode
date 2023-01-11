// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TurretPIDSubsystem;
import frc.robot.Constants.TurretConstants;

public class LookForTarget extends CommandBase {
  private TurretPIDSubsystem m_turretPIDsubsystem;
  /** Creates a new LookForTarget. */
  public LookForTarget(TurretPIDSubsystem m_turretPIDsubsystem) {
    this.m_turretPIDsubsystem = m_turretPIDsubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_turretPIDsubsystem.disable();
    m_turretPIDsubsystem.setMotor(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_turretPIDsubsystem.turretPositionDEG() > TurretConstants.turretLeftWarning) {
      m_turretPIDsubsystem.setMotor(-1);
    }

    if (m_turretPIDsubsystem.turretPositionDEG() < TurretConstants.turretRightWarning) {
      m_turretPIDsubsystem.setMotor(1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turretPIDsubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
