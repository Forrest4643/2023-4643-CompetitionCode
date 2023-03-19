// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.armConstants;

public class ArmSubsystem extends SubsystemBase {

  private final CANSparkMax m_armMotor = new CANSparkMax(armConstants.kArmID, MotorType.kBrushless);

  private final SparkMaxAbsoluteEncoder m_armEncoder = m_armMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private final SparkMaxPIDController m_armController = m_armMotor.getPIDController();

  private static double m_kP = 0.003; //TODO tune arm PID

  private static double m_kI = 0.001;
 
  private static double m_kD = 0;

  private static double m_kF = 0.01;

  private static double allowedErrorDEG = 3;

  private double m_armReferencePointDEG;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
   
  //m_armEncoder.setPositionConversionFactor(123.75); //TODO is this right?
  //m_armEncoder.setZeroOffset(118.0008674);
  m_armEncoder.setInverted(false);

  m_armMotor.setInverted(false);

  m_armController.setFeedbackDevice(m_armEncoder);
  m_armController.setP(m_kP);
  m_armController.setI(m_kI);
  m_armController.setD(m_kD);
  m_armController.setFF(m_kF);

  m_armController.setSmartMotionMaxAccel(150, 0);
  m_armController.setSmartMotionMaxVelocity(60, 0);
  m_armController.setSmartMotionAllowedClosedLoopError(allowedErrorDEG, 0);

  m_armController.setIZone(3, 0);

  m_armMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
  m_armMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
  m_armMotor.burnFlash();

  }

  @Override
  public void periodic() {
   
    SmartDashboard.putNumber("armEncoderPosition", armEncoderPosition());
    SmartDashboard.putNumber("armMotorPosition", m_armMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("armSetpoint", m_armReferencePointDEG);
    SmartDashboard.putBoolean("armAtSetpoint?", atSetpoint());
  }

  public boolean atSetpoint() {
    return Math.abs(m_armEncoder.getPosition() - m_armReferencePointDEG) < allowedErrorDEG;
  }
  
  public void setArmReferenceDEG(double referenceDEG) {
    m_armReferencePointDEG = referenceDEG;
    m_armController.setReference(m_armReferencePointDEG, ControlType.kSmartMotion);
    System.out.println("Arm Reference Updated! ReferenceDEG:" + m_armReferencePointDEG);
    //System.out.println("Arm Reported Position:" + m_armEncoder.getPosition());

  }

  public void unStow1() {
    m_armReferencePointDEG = 45;
    System.out.println("unStow1!");
  }

  public void unStow2() {
    m_armReferencePointDEG = 0;
    System.out.println("unStow2!");

  }

  public void matchStow() {
    m_armReferencePointDEG = 0;
    System.out.println("matchStow!");
  }

  public void armHorizontal() {
    m_armReferencePointDEG = 70;
    System.out.println("armHorizontal!");
  }

  public double armEncoderPosition() {
    return (m_armEncoder.getPosition());
  }
}
