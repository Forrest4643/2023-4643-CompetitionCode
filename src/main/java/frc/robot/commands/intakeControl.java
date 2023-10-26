// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.schedulers.SequentialScheduler;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.driveConstants;
import frc.robot.Constants.wristConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopingSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.subsystems.MandibleSubsystem;

public class intakeControl extends CommandBase {

  private WristSubsystem m_wristSubsystem;
  private ArmSubsystem m_armSubsystem;
  private TelescopingSubsystem m_telescopingSubsystem;
  private MandibleSubsystem m_mandibleSubsystem;
  private XboxController m_operateController;

  /** Creates a new intakeControl. */
  public intakeControl(ArmSubsystem m_ArmSubsystem, WristSubsystem m_WristSubsystem, TelescopingSubsystem m_TelescopingSubsystem, MandibleSubsystem m_MandibleSubsystem, XboxController m_OperateController) {

    this.m_wristSubsystem = m_WristSubsystem;
    this.m_armSubsystem = m_ArmSubsystem;
    this.m_telescopingSubsystem = m_TelescopingSubsystem;
    this.m_mandibleSubsystem = m_MandibleSubsystem;
    this.m_operateController = m_OperateController;


    addRequirements(m_wristSubsystem, m_armSubsystem, m_telescopingSubsystem, m_mandibleSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wristSubsystem.matchStow();
    m_telescopingSubsystem.matchStow();
    m_mandibleSubsystem.intakeHold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_operateController.getBButtonPressed();
  }
}
