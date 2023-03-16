// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;



public class unStow extends CommandBase {
  ArmSubsystem m_armSubsystem;
  WristSubsystem m_wristSubsystem;

  private static final double m_armPos1DEG = 36;
  private static final double m_armPos2 = 78.5;
  private static final double m_wristPosDEG = 80;

  /** Creates a new unStow. */
  public unStow(ArmSubsystem m_ArmSubsystem, WristSubsystem m_WristSubsystem) {
    this.m_armSubsystem = m_ArmSubsystem;
    this.m_wristSubsystem = m_WristSubsystem;
    addRequirements(m_armSubsystem, m_wristSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
