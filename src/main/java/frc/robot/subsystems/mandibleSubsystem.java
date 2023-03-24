// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.mandibleConstants;

public class mandibleSubsystem extends SubsystemBase {
  
  private final CANSparkMax m_mandibleMotor = new CANSparkMax(mandibleConstants.kMandibleID, MotorType.kBrushless);

  /** Creates a new mandibleSubsystem. */
  public mandibleSubsystem() {
    m_mandibleMotor.enableVoltageCompensation(12);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("intakeCurrentSpike", intakeCurrentSpike());
    SmartDashboard.putNumber("intakeCurrent", getIntakeCurrent());
  }

  public void intakeFull() {
    m_mandibleMotor.setVoltage(12);
  }

  public void intakeHold() {
    m_mandibleMotor.setVoltage(mandibleConstants.kHoldVoltage);
  }

  public void shootFull() {
    m_mandibleMotor.setVoltage(-12);
  }

  public void shootHalf() {
    m_mandibleMotor.setVoltage(-6);
  }

  public boolean intakeCurrentSpike() {
    return m_mandibleMotor.getOutputCurrent() > mandibleConstants.kCurrentThresh;
  }
  public double getIntakeCurrent() {
    return m_mandibleMotor.getOutputCurrent();
  }

  public boolean intakeAtSpeed() {
    return m_mandibleMotor.getEncoder().getVelocity() > 2000;
  }


}
