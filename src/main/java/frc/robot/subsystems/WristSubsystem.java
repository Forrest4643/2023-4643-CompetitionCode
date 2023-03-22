// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.wristConstants;
import io.github.oblarg.oblog.Loggable;

public class WristSubsystem extends SubsystemBase implements Loggable {
 
  private static double m_kP = 0.008;
  private static double m_kI = 0.00008;
  private static double m_kD = 0.01;

  private static double allowedErrorDEG = 1;

  private double m_wristReferencePointDEG = 0;


  private final CANSparkMax m_wristMotor = new CANSparkMax(wristConstants.kWristID, MotorType.kBrushless);

  private final SparkMaxAbsoluteEncoder m_wristEncoder = m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private final SparkMaxPIDController m_wristController = m_wristMotor.getPIDController();

  /** Creates a new wristSubsystem. */
  public WristSubsystem() {

    m_wristEncoder.setPositionConversionFactor(360);

    m_wristEncoder.setZeroOffset(157.0082331); 

    m_wristController.setFeedbackDevice(m_wristEncoder);

    m_wristController.setP(m_kP);
    m_wristController.setI(m_kI);
    m_wristController.setD(m_kD);

    m_wristController.setIZone(5);

    m_wristMotor.burnFlash();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("wristPosition:", m_wristEncoder.getPosition());
    SmartDashboard.putBoolean("wristAtSetpoint?", atSetpoint());
    SmartDashboard.putNumber("wristSetpoint", m_wristReferencePointDEG);
  }

  public double getWristPosition() {
    return m_wristEncoder.getPosition();
  }

  public boolean atSetpoint() {
    return Math.abs(m_wristEncoder.getPosition() - m_wristReferencePointDEG) < allowedErrorDEG;
  }

  public void setWristReference(double referenceDEG) {
    m_wristReferencePointDEG = referenceDEG;
    m_wristController.setReference(m_wristReferencePointDEG, ControlType.kSmartMotion);
  }
 
  public void unStow() {
    setWristReference(200);
  }

  public void matchStow() {
    setWristReference(200);
  }

  public void setHorizontal() {
    setWristReference(120);
  }
}
