// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.mandibleConstants;

public class mandibleSubsystem extends SubsystemBase {
  
  private final CANSparkMax m_mandibleMotor = new CANSparkMax(mandibleConstants.kMandibleID, MotorType.kBrushless);

  /** Creates a new mandibleSubsystem. */
  public mandibleSubsystem() {
    m_mandibleMotor.enableVoltageCompensation(12);
    m_mandibleMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
 
  }

  public void intake() {
    m_mandibleMotor.setVoltage(4);
  }

  public void intakeHold() {
    m_mandibleMotor.setVoltage(mandibleConstants.kHoldVoltage);
  }

  public void shootFull() {
    m_mandibleMotor.setVoltage(-3);
  }

  public void shootHalf() {
    m_mandibleMotor.setVoltage(-6);
  }

  public void stopMotors() {
    m_mandibleMotor.setVoltage(0);
  }

  public boolean intakeCurrentSpike() {
    return m_mandibleMotor.getOutputCurrent() > mandibleConstants.kCurrentThresh;
  }
  public double getIntakeCurrent() {
    return m_mandibleMotor.getOutputCurrent();
  }

  public boolean intakeAtSpeed() {
    return m_mandibleMotor.getEncoder().getVelocity() > 10000;
  }


}
