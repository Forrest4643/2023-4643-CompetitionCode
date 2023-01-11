// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DriveDistance extends CommandBase {

  private DriveSubsystem m_driveSubsystem;
  private double m_inches;
  private PIDController m_drivePID = new PIDController(DriveConstants.drivekP, DriveConstants.drivekI,
      DriveConstants.drivekD);

  /** Creates a new DriveDistance. */
  public DriveDistance(DriveSubsystem m_driveSubsystem, double m_inches) {
    this.m_driveSubsystem = m_driveSubsystem;
    this.m_inches = m_inches;
    addRequirements(m_driveSubsystem);
    m_drivePID.setTolerance(0.1);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivePID.setSetpoint(m_driveSubsystem.getAverageEncoderDistance() + m_inches);
    System.out.println("driveDist Start!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSubsystem.setDrive(MathUtil.clamp(-m_drivePID.calculate(m_driveSubsystem.getAverageEncoderDistance()), -.5, .5), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("DriveDistance End!" + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //return false;
    return m_drivePID.atSetpoint();
  }
}
