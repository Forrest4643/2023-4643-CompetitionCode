// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax m_cMotor = new CANSparkMax(ClimberConstants.climbID, MotorType.kBrushless);
  private final RelativeEncoder m_cEncoder = m_cMotor.getEncoder();
  
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    m_cMotor.setIdleMode(IdleMode.kBrake);
    m_cEncoder.setPositionConversionFactor(ClimberConstants.conversionFactor);
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("climberInches", climberInches());
    // This method will be called once per scheduler run
  }

  public void setMotor(double speed) {
    double limitedOutput;
    if (climberInches() < ClimberConstants.minLimit) {
      limitedOutput = MathUtil.clamp(speed, -1, 0);
      //System.out.println("FWDLIMIT");
    } else if (climberInches()> ClimberConstants.maxLimit) {
      limitedOutput = MathUtil.clamp(speed, 0, 1);
      //System.out.println("REVLIMIT");
    } else {
      limitedOutput = speed;
    }
    m_cMotor.set(speed);
  }

  public void up() {
    setMotor(-1);
  }

  public void down() {
    setMotor(1);
  }

  public void idle() {
    setMotor(0);
  }

  public double climberInches() {
    return (m_cEncoder.getPosition());
  }
}

//-14840 = max height