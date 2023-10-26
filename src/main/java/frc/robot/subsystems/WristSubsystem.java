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

public class WristSubsystem extends SubsystemBase {

  private ArmSubsystem m_armSubsystem;
 
  private double m_kP = 0.004;
  private double m_kI = 0.0;
  private double m_kD = 0.001;

  private static final double m_wristOffsetDEG = 115;

  private double allowedErrorDEG = 7;

  private double m_wristReferencePointDEG = 0;

  private double m_kG = 0.1;

  private double m_iZone = 1;

  private int m_currentLimit = 15;


  private final CANSparkMax m_wristMotor = new CANSparkMax(wristConstants.kWristID, MotorType.kBrushless);

  private final SparkMaxAbsoluteEncoder m_wristEncoder = m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private final SparkMaxPIDController m_wristController = m_wristMotor.getPIDController();

  private ArmFeedforward m_wristFF = new ArmFeedforward(0, m_kG, 0);

  /** Creates a new wristSubsystem. */
  public WristSubsystem(ArmSubsystem m_ArmSubsystem) {

    this.m_armSubsystem = m_ArmSubsystem;

    SmartDashboard.putNumber("Wrist_kP", m_kP);
    SmartDashboard.putNumber("Wrist_kI", m_kI);
    SmartDashboard.putNumber("wrist_kD", m_kD);
    SmartDashboard.putNumber("wrist_kG", m_kG);
    SmartDashboard.putNumber("wrist_iZone", m_iZone);

    m_wristEncoder.setPositionConversionFactor(367.34);

    m_wristEncoder.setZeroOffset(219); 

    m_wristController.setFeedbackDevice(m_wristEncoder);

    m_wristController.setOutputRange(-1,1);

    m_wristController.setP(m_kP);
    m_wristController.setI(m_kI);
    m_wristController.setD(m_kD);

    m_wristController.setIZone(m_iZone);

    m_wristMotor.setSmartCurrentLimit(m_currentLimit);

    m_wristMotor.setInverted(false);

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
     0, m_wristFF.calculate(Units.degreesToRadians(m_wristReferencePointDEG + m_armSubsystem.armEncoderPosition()), 0)); //TODO is encoder noise causing wrist oscillations? 
  }
 
  public void unStow() {
    setWristReference(-90);
  }

  public void holdForUnstow() {
    setWristReference(75);
  }

  public void matchStow() {
    setWristReference(-90);
  }

  public void intakePosition() {
    setWristReference(0);
  }

  public void setHorizontal() {
    setWristReference(0);
  }

  public void updateWristSmartDashValues() {
    //updates PIDG values to what is on the smartdashboard 
    double sumValuesBefore = m_kP+m_kI+m_kD+m_kG+m_iZone;

    m_kP = SmartDashboard.getNumber("Wrist_kP", m_kP);
    m_kI = SmartDashboard.getNumber("Wrist_kI", m_kI);
    m_kD = SmartDashboard.getNumber("wrist_kD", m_kD);
    m_kG = SmartDashboard.getNumber("wrist_kG", m_kG);
    m_iZone = SmartDashboard.getNumber("wrist_iZone", m_iZone);

    double sumValuesAfter = m_kP+m_kI+m_kD+m_kG+m_iZone;

    //checks if any values changed, if true update spark controller
    if(sumValuesBefore != sumValuesAfter) {
    //applies PID to wrist spark controller 
    m_wristController.setP(m_kP);
    m_wristController.setI(m_kI);
    m_wristController.setD(m_kD);

    System.out.println("WRIST PID UPDATED!"+"kP="+m_kP+"kI="+m_kI+"kD="+m_kD+"kG="+m_kG+"iZone="+m_iZone);
    }
  }
}

