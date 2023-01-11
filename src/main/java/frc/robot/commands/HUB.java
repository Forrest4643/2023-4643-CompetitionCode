// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.ShooterPIDSubsystem;
import frc.robot.subsystems.TurretPIDSubsystem;

public class HUB extends CommandBase {
  private ShooterPIDSubsystem m_shooterPIDsubsystem;
  private HoodPIDSubsystem m_hoodPIDsubsystem;
  private TurretPIDSubsystem m_turretPIDsubsystem;
  private TurretPosition m_turretposition = new TurretPosition(m_turretPIDsubsystem, TurretConstants.HUBposition);  
  /** Creates a new HUB. */
  public HUB(TurretPosition m_turretposition, ShooterPIDSubsystem m_shooterPIDsubsystem, HoodPIDSubsystem m_hoodPIDsubsystem, TurretPIDSubsystem m_turretPIDsubsystem) {
    this.m_turretposition = m_turretposition;
    this.m_shooterPIDsubsystem = m_shooterPIDsubsystem;
    this.m_hoodPIDsubsystem = m_hoodPIDsubsystem;
    this.m_turretPIDsubsystem = m_turretPIDsubsystem;
    addRequirements(m_shooterPIDsubsystem, m_hoodPIDsubsystem, m_turretPIDsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_hoodPIDsubsystem.enable();
    m_shooterPIDsubsystem.enable();
    SmartDashboard.putBoolean("Shooter", true);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!m_shooterPIDsubsystem.isEnabled()) {
      m_shooterPIDsubsystem.enable();
    }

    if(!m_hoodPIDsubsystem.isEnabled()) {
      m_hoodPIDsubsystem.enable();
    }

    m_shooterPIDsubsystem.setSetpoint(ShooterConstants.hubShot);
    m_hoodPIDsubsystem.setSetpoint(HoodConstants.hubShot);
    if (!m_turretposition.isScheduled()) {
      m_turretposition.schedule();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooterPIDsubsystem.disable();
    m_hoodPIDsubsystem.disable();
    if(m_turretposition.isScheduled()){
    m_turretposition.cancel();
    }
    SmartDashboard.putBoolean("Shooter", false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
