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
 
  private double m_kP = 0.001;
  private double m_kI = 0.00;
  private double m_kD = 0.0;

  private static final double m_wristOffsetDEG = -225;

  private double allowedErrorDEG = 7;

  private double m_wristReferencePointDEG = 0;

  private double m_kG = 3;

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
    SmartDashboard.putNumber("Wrist_kD", m_kD);
    SmartDashboard.putNumber("Wrist_kG", m_kG);
    SmartDashboard.putNumber("Wrist_iZone", m_iZone);

    m_wristEncoder.setPositionConversionFactor(360);

    m_wristEncoder.setInverted(false);

    m_wristEncoder.setZeroOffset(28); 

    m_wristController.setFeedbackDevice(m_wristEncoder);

    m_wristController.setOutputRange(-1,1);

    m_wristController.setP(m_kP);
    m_wristController.setI(m_kI);
    m_wristController.setD(m_kD);

    m_wristController.setIZone(m_iZone);

    m_wristMotor.setSmartCurrentLimit(m_currentLimit);

    m_wristMotor.setInverted(true);

    m_wristMotor.burnFlash();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("wristPosition:", getWristPosition());
    SmartDashboard.putBoolean("wristAtSetpoint?", atSetpoint());
    SmartDashboard.putNumber("wristSetpoint", m_wristReferencePointDEG);
    SmartDashboard.putNumber("absWristPos", absWristPositionDEG());

    
  }

  public double getWristPosition() {
    return m_wristEncoder.getPosition() + m_wristOffsetDEG;
  }

  public double absWristPositionDEG() {
    return (getWristPosition() + m_armSubsystem.armEncoderPosition());
  }

  

  public boolean atSetpoint() {
    return Math.abs(getWristPosition() - m_wristReferencePointDEG) < allowedErrorDEG;
  }

  public void setWristReference(double referenceDEG) {
    m_wristReferencePointDEG = referenceDEG;
    m_wristController.setReference((m_wristReferencePointDEG - m_wristOffsetDEG), ControlType.kSmartMotion,
     0, m_wristFF.calculate(Units.degreesToRadians(absWristPositionDEG()), 0)); 
  }
 
  public void unStow() {
    setWristReference(wristConstants.kUnStow);
  }

  public void holdForUnstow() {
    setWristReference(wristConstants.kHoldForUnstow);
  }

  public void matchStow() {
    setWristReference(wristConstants.kMatchStow);
  }

  public void intakePosition() {
    setWristReference(wristConstants.kIntake);
  }

  public void Horizontal() {
    setWristReference(wristConstants.kHorizontal);
  }

  public void lowCube() {
    setWristReference(wristConstants.kScoreLowCubePos);
  }

  public void midCube() {
    setWristReference(wristConstants.kScoreMidCubePos);
  }

  public void highCube() {
    setWristReference(wristConstants.kScoreHighCubePos);
  }

  public void lowCone() {
    setWristReference(wristConstants.kScoreLowConePos);
  }

  public void midCone() {
    setWristReference(wristConstants.kScoreMidConePos);
  }

  public void substationIntake() {
    setWristReference(wristConstants.kSubstationPos);
  }
  
  public void updateWristSmartDashValues() {
    //updates PIDG values to what is on the smartdashboard 
    double sumValuesBefore = m_kP+m_kI+m_kD+m_kG+m_iZone;

    m_kP = SmartDashboard.getNumber("Wrist_kP", m_kP);
    m_kI = SmartDashboard.getNumber("Wrist_kI", m_kI);
    m_kD = SmartDashboard.getNumber("Wrist_kD", m_kD);
    m_kG = SmartDashboard.getNumber("Wrist_kG", m_kG);
    m_iZone = SmartDashboard.getNumber("Wrist_iZone", m_iZone);

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

