// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PneumaticsSubsystem;

public class AutoIndex extends CommandBase {
  private IntakeSubsystem m_intakeSubsystem;
  private IndexerSubsystem m_indexerSubsystem;
  private PneumaticsSubsystem m_pneumaticsSubsystem;
  private Sensors m_sensors;
  private BooleanSupplier m_forward;
  private BooleanSupplier m_reverse;
  private BooleanSupplier m_intakereverse;
  private IntSupplier m_POV;
  private boolean m_toggled;

  /** Creates a new AutoIndex. */
  public AutoIndex(IntakeSubsystem m_intakeSubsystem, IndexerSubsystem m_indexerSubsystem,
      PneumaticsSubsystem m_pneumaticsSubsystem, Sensors m_sensors, XboxController m_operateController) {
    this.m_intakeSubsystem = m_intakeSubsystem;
    this.m_indexerSubsystem = m_indexerSubsystem;
    this.m_pneumaticsSubsystem = m_pneumaticsSubsystem;
    this.m_sensors = m_sensors;

    m_POV = () -> m_operateController.getPOV();

    m_forward = () -> m_operateController.getRightBumper();
    m_reverse = () -> m_operateController.getLeftBumper();

    m_intakereverse = () -> m_operateController.getBackButton();

    addRequirements(m_intakeSubsystem, m_indexerSubsystem);
  }

  BangBangController m_indexBangController = new BangBangController();

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_indexerSubsystem.Front.setIdleMode(IdleMode.kBrake);
    m_indexerSubsystem.Rear.setIdleMode(IdleMode.kBrake);
    m_indexBangController.setTolerance(IndexerConstants.bangTolerance);
    m_toggled = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_sensors.navXPitch() < 10 && m_sensors.navXPitch() > -10) {
      if (m_POV.getAsInt() == 90) {
        m_toggled = true;
      }
      if (m_POV.getAsInt() == 270) {
        m_toggled = false;

      }

      if (m_toggled) {
        manual();
      } else {
        manual();
        //auto();
      }
    } else {
      dontDie();
    }

    SmartDashboard.putBoolean("manual indexing", m_toggled);
  }

  private void manual() {

    if (m_pneumaticsSubsystem.rearIntakeStatus()) {
      if (m_POV.getAsInt() == 180) {
        m_intakeSubsystem.rearWheelsReverse();
      } else if (m_POV.getAsInt() != 180){
        m_intakeSubsystem.rearWheelsOn();
      }
    } else {
      m_intakeSubsystem.rearWheelsOff();
    }

    if (m_pneumaticsSubsystem.frontIntakeStatus()) {
      if (m_POV.getAsInt() == 180) {
        m_intakeSubsystem.frontWheelsReverse();
      } else if (m_POV.getAsInt() != 180){
        m_intakeSubsystem.frontWheelsOn();
      }
    } else {
      m_intakeSubsystem.frontWheelsOff();
    }

    if (m_forward.getAsBoolean()) {
      m_indexerSubsystem.wheelsOn();
    } else if (m_reverse.getAsBoolean()) {
      m_indexerSubsystem.wheelsReverse();
    } else {
      m_indexerSubsystem.wheelsOff();
    }
  }

  // private void auto() {
  //   if (m_pneumaticsSubsystem.rearStatus()) {
  //     m_intakeSubsystem.rearWheelsOn();
  //   } else {
  //     m_intakeSubsystem.rearWheelsOff();
  //   }

  //   if (m_pneumaticsSubsystem.frontStatus()) {
  //     m_intakeSubsystem.frontWheelsOn();
  //   } else {
  //     m_intakeSubsystem.frontWheelsOff();
  //   }
  // }

  private void dontDie() {
      while(Math.abs(m_sensors.navXPitch()) > 20) {
      m_pneumaticsSubsystem.rearIntakeOpen();
      m_pneumaticsSubsystem.frontIntakeOpen();
      }
      m_pneumaticsSubsystem.frontIntakeClosed();
      m_pneumaticsSubsystem.rearIntakeClosed();
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intakeSubsystem.frontWheelsOff();
    m_intakeSubsystem.rearWheelsOff();
    m_indexerSubsystem.wheelsOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
