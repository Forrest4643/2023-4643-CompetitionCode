// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.HoodPIDSubsystem;
import frc.robot.subsystems.ShooterPIDSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAim extends CommandBase {
  private VisionSubsystem m_visionsubsystem;
  private ShooterPIDSubsystem m_shooterPIDsubsystem;
  private HoodPIDSubsystem m_hoodPIDsubsystem;

  /** Creates a new driveAim. */
  public AutoAim(HoodPIDSubsystem m_hoodPIDsubsystem, VisionSubsystem m_visionsubsystem,
      ShooterPIDSubsystem m_shooterPIDsubsystem) {
    this.m_visionsubsystem = m_visionsubsystem;
    this.m_shooterPIDsubsystem = m_shooterPIDsubsystem;
    this.m_hoodPIDsubsystem = m_hoodPIDsubsystem;

    addRequirements(m_hoodPIDsubsystem, m_shooterPIDsubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("AutoAim Started!");
    SmartDashboard.putBoolean("Shooter", true);
    // enable PID outputs
    m_shooterPIDsubsystem.enable();
    m_hoodPIDsubsystem.enable();
    // turn vision LEDS on
    m_visionsubsystem.LEDon();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (m_visionsubsystem.hasTargets()) {
      aim();
    } else {
      m_hoodPIDsubsystem.setSetpoint(65);
      m_shooterPIDsubsystem.setSetpoint(0);
    }
  }

  //sets shooter RPM and hood angle based on distance
  private void aim() {
    double targetDistance = m_visionsubsystem.getTargetDistanceIN() / 12;

    // setting hood launch angle based off of 2nd degree polynomial
    m_hoodPIDsubsystem.setSetpoint(MathUtil.clamp(
        HoodConstants.quadAimC +
            (HoodConstants.quadAimA * targetDistance) +
            (Math.pow(targetDistance, 2) * HoodConstants.quadAimB),
        HoodConstants.highAngleLimit, HoodConstants.lowAngleLimit));

    // setting shooter RPM based off of 3rd degree polynomial
    m_shooterPIDsubsystem.setSetpoint(ShooterConstants.quadAimD +
        (ShooterConstants.quadAimA * targetDistance) +
        (Math.pow((targetDistance), 2) * ShooterConstants.quadAimB) +
        (Math.pow(targetDistance, 3) * ShooterConstants.quadAimC));

    // debug info
    SmartDashboard.putNumber("hoodPositionDEG", m_hoodPIDsubsystem.getHoodPositionDEG());
    SmartDashboard.putNumber("hoodSetpoint", m_hoodPIDsubsystem.getSetpoint());
    SmartDashboard.putNumber("leftAmps", m_shooterPIDsubsystem.leftAmps());
    SmartDashboard.putNumber("rightAmps", m_shooterPIDsubsystem.rightAmps());
    SmartDashboard.putNumber("shooterRPM", m_shooterPIDsubsystem.getShooterRPM());
    SmartDashboard.putBoolean("readyFire", readyFire());
  }

 

  public boolean readyFire() {
    // when all of the PID controllers are at their setpoints, system is ready to
    // fire.
    return m_shooterPIDsubsystem.getController().atSetpoint()
        && m_hoodPIDsubsystem.getController().atSetpoint();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // disable PID outputs to prevent unnecessary movement
    m_shooterPIDsubsystem.disable();
    m_hoodPIDsubsystem.disable();

    SmartDashboard.putBoolean("Shooter", false);

    // debug info
    System.out.println("AutoAim Ended!");
  }

}
