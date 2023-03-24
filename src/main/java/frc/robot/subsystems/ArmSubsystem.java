// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armConstants;
import frc.robot.Constants.telescopingConstant;

public class ArmSubsystem extends SubsystemBase {

  private TelescopingSubsystem m_telescopingSubsystem;

  private final CANSparkMax m_armMotor = new CANSparkMax(armConstants.kArmID, MotorType.kBrushless);

  private final RelativeEncoder m_armMotorEncoder = m_armMotor.getAlternateEncoder(8192);

  //TODO uncomment when breakout board arrives
  //private final SparkMaxAbsoluteEncoder m_armEncoder = m_armMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private final SparkMaxPIDController m_armController = m_armMotor.getPIDController();

  private ArmFeedforward m_armFeedforward; 

  private static double m_kP = 0.0055; //TODO tune arm PID

  private static double m_kI = 0.000075;
 
  private static double m_kD = 0.0;

  private static double m_kF = 0.01;

  private double m_kG;

  private static final double m_kGmin = 0.65;
  private static final double m_kGmax = 1.04;

  private double m_kGmultiplier = m_kGmax / m_kGmin; //retracted kG over extended kG

  private static double m_armEncoderOffset = -115;

  private static double allowedErrorDEG = 3;

  private double m_armReferencePointDEG;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(TelescopingSubsystem m_TelescopingSubsystem) {
   this.m_telescopingSubsystem = m_TelescopingSubsystem;

  //hall effect internal: 0.34375
  //alternate thru bore: 123.75
  m_armMotorEncoder.setPositionConversionFactor(123.75);
  m_armMotorEncoder.setInverted(true);
  m_armMotor.setInverted(false);

  m_armMotor.setSmartCurrentLimit(35);

  m_armMotor.getEncoder().setPositionConversionFactor(0.34375);
  //TODO change to absolute encoder when breakout board arrives
  m_armController.setFeedbackDevice(m_armMotorEncoder);

  //TODO uncomment when breakout board arrives
  //m_armEncoder.setInverted(false);
  //m_armEncoder.setPositionConversionFactor(123.75); //TODO is this right?
  //m_armEncoder.setZeroOffset(118.0008674);


  m_armController.setP(m_kP);
  m_armController.setI(m_kI);
  m_armController.setD(m_kD);
  m_armController.setFF(m_kF);

  m_armController.setSmartMotionMaxAccel(400, 0);
  m_armController.setSmartMotionMaxVelocity(40, 0);
  m_armController.setSmartMotionAllowedClosedLoopError(2, 0);

  m_armController.setIZone(3, 0);

  m_armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
  m_armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

  m_armMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
  m_armMotor.setSoftLimit(SoftLimitDirection.kForward, 130);

  m_armMotor.burnFlash();

  }

  @Override
  public void periodic() {
   
    SmartDashboard.putNumber("armEncoderPosition", armEncoderPosition());
    SmartDashboard.putNumber("armMotorPosition", m_armMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("armSetpoint", m_armReferencePointDEG);
    SmartDashboard.putBoolean("armAtSetpoint?", atSetpoint());

    m_kG = ((m_telescopingSubsystem.telescopingPosIN() / telescopingConstant.kMaxPositionIN)
     * m_kGmultiplier);
    
    m_armFeedforward = new ArmFeedforward(0, m_kG, 0);

  }

  public boolean atSetpoint() {
    return Math.abs(m_armMotorEncoder.getPosition() - m_armReferencePointDEG) < allowedErrorDEG;
  }
  
  public void setArmReferenceDEG(double referenceDEG) {
    m_armReferencePointDEG = referenceDEG;
    m_armController.setReference(m_armReferencePointDEG, ControlType.kSmartMotion, 0, 
      m_armFeedforward.calculate(Units.degreesToRadians(m_armReferencePointDEG), 0));
    System.out.println("Arm Reference Updated! ReferenceDEG:" + m_armReferencePointDEG);
    System.out.println("Arm Reported Position:" + m_armMotorEncoder.getPosition());

  }

  public void unStow1() {
    setArmReferenceDEG(-45);
    System.out.println("unStow1!");
  }

  public void unStow2() {
    setArmReferenceDEG(-50);
    System.out.println("unStow2!");

  }

  public void matchStow() {
  setArmReferenceDEG(-50);
  System.out.println("matchStow!");
  }

  public void armHorizontal() {
    setArmReferenceDEG(0);;
    System.out.println("armHorizontal!");
  }

  public double armEncoderPosition() {
    return (m_armMotorEncoder.getPosition() - m_armEncoderOffset);
  }
}
