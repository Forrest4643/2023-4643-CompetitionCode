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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.wristConstants;
import io.github.oblarg.oblog.Loggable;

public class WristSubsystem extends SubsystemBase implements Loggable {

  private ArmSubsystem m_armSubsystem;
 
  private static double m_kP = 0.001;
  private static double m_kI = 0.0;
  private static double m_kD = 0.001;

  private static final double m_wristOffsetDEG = 115;

  private static double allowedErrorDEG = 5;

  private double m_wristReferencePointDEG = 0;




  private final CANSparkMax m_wristMotor = new CANSparkMax(wristConstants.kWristID, MotorType.kBrushless);

  private final SparkMaxAbsoluteEncoder m_wristEncoder = m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private final SparkMaxPIDController m_wristController = m_wristMotor.getPIDController();

  private ArmFeedforward m_wristFF = new ArmFeedforward(0, 0.1, 0);

  /** Creates a new wristSubsystem. */
  public WristSubsystem(ArmSubsystem m_ArmSubsystem) {

    this.m_armSubsystem = m_ArmSubsystem;

    m_wristEncoder.setPositionConversionFactor(360);

    m_wristEncoder.setZeroOffset(342.8082204); 

    m_wristController.setFeedbackDevice(m_wristEncoder);

    m_wristController.setP(m_kP);
    m_wristController.setI(m_kI);
    m_wristController.setD(m_kD);

    m_wristController.setIZone(1);

    m_wristMotor.setSmartCurrentLimit(10);

    m_wristMotor.burnFlash();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("wristPosition:", getWristPosition());
    SmartDashboard.putBoolean("wristAtSetpoint?", atSetpoint());
    SmartDashboard.putNumber("wristSetpoint", m_wristReferencePointDEG);
  }

  public double getWristPosition() {
    return -m_wristEncoder.getPosition() + m_wristOffsetDEG;
  }

  public boolean atSetpoint() {
    return Math.abs(getWristPosition() - m_wristReferencePointDEG) < allowedErrorDEG;
  }

  public void setWristReference(double referenceDEG) {
    m_wristReferencePointDEG = referenceDEG;
    m_wristController.setReference((-m_wristReferencePointDEG) + m_wristOffsetDEG, ControlType.kSmartMotion,
     0, m_wristFF.calculate(Units.degreesToRadians(m_wristReferencePointDEG + m_armSubsystem.armEncoderPosition()), 0));
  }
 
  public void unStow() {
    setWristReference(-60);
  }

  public void matchStow() {
    setWristReference(-60);
  }

  public void intakePosition() {
    setWristReference(-10);
  }

  public void setHorizontal() {
    setWristReference(0);
  }
}
