// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

  private SparkMaxPIDController m_armController = m_armMotor.getPIDController();

  private ArmFeedforward m_armFeedforward; 

  private double m_kP = 0.004; 

  private double m_kI = 0.0;
 
  private double m_kD = 0.001;

  private double m_kF = 0.0;

  private double m_kG;

  private double m_kGmin = 0.9; //kG when telescoping is retracted
  private double m_kGmax = 1.5; //kG wgeb telescoping is extended

  private double m_kGmultiplier = m_kGmax / m_kGmin; //retracted kG over extended kG

  private static double m_armEncoderOffset = -80;

  private static double allowedErrorDEG = 5;

  private double m_armReferencePointDEG;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(TelescopingSubsystem m_TelescopingSubsystem) {
   this.m_telescopingSubsystem = m_TelescopingSubsystem;

  //hall effect internal: 0.34375
  //alternate thru bore: 123.75
  m_armMotorEncoder.setPositionConversionFactor(123.75);
  m_armMotorEncoder.setInverted(true);
  m_armMotor.setInverted(false);

  m_armMotor.setSmartCurrentLimit(45);

  m_armMotor.getEncoder().setPositionConversionFactor(0.34375);
  //TODO change to absolute encoder when breakout board arrives
  m_armController.setFeedbackDevice(m_armMotorEncoder);

  //TODO uncomment when breakout board arrives
  //m_armEncoder.setInverted(false);
  //m_armEncoder.setPositionConversionFactor(123.75); //TODO is this right?
  //m_armEncoder.setZeroOffset(118.0008674);

  SmartDashboard.putNumber("Arm_kP", m_kP);
  SmartDashboard.putNumber("Arm_kI", m_kI);
  SmartDashboard.putNumber("Arm_kD", m_kD);
  SmartDashboard.putNumber("Arm_kG_MIN", m_kGmin);
  SmartDashboard.putNumber("Arm_kG_MAX", m_kGmax);

  
  m_armController.setP(m_kP);
  m_armController.setI(m_kI);
  m_armController.setD(m_kD);
  m_armController.setFF(m_kF);

  m_armController.setSmartMotionMaxAccel(400, 0);
  m_armController.setSmartMotionMaxVelocity(150, 0);
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
    SmartDashboard.putNumber("armSetpoint", m_armReferencePointDEG);
    SmartDashboard.putBoolean("armAtSetpoint?", atSetpoint());

    

    //System.out.println("Arm Reported Position:" + m_armMotorEncoder.getPosition());

  }

  public boolean atSetpoint() {
    return Math.abs(-armEncoderPosition() - m_armReferencePointDEG) < allowedErrorDEG;
  }

  public BooleanSupplier atSetpointSupplier() {
    return () -> atSetpoint();
  }
  
  public void setArmReferenceDEG(double referenceDEG) {
    m_armReferencePointDEG = referenceDEG;
    m_armController.setReference(m_armReferencePointDEG - m_armEncoderOffset, ControlType.kSmartMotion, 0, 
      arbFFVolts());
    System.out.println("Arm Reference Updated! ReferenceDEG:" + m_armReferencePointDEG);
  }

  public void unStow1() {
    setArmReferenceDEG(-20);
    System.out.println("unStow1!");
  }

  public void unStow2() {
    setArmReferenceDEG(-80);
    System.out.println("unStow2!");

  }

  public void matchStow() {
    setArmReferenceDEG(-70);
    System.out.println("matchStow!");
  }

  public void intakePosition() {
    setArmReferenceDEG(-35);
  }

  public void armHorizontal() {
    setArmReferenceDEG(0);;
    System.out.println("armHorizontal!");
  }

  public void substationIntake() {
    setArmReferenceDEG(armConstants.kSubstationPos);
  } 

  public double armEncoderPosition() {
    return (-m_armMotorEncoder.getPosition() - m_armEncoderOffset);
  }

  public double arbFFVolts() {

    m_kG = ((MathUtil.clamp(m_telescopingSubsystem.telescopingPosIN(), 1, telescopingConstant.kMaxPositionIN)
    / telescopingConstant.kMaxPositionIN)
    * m_kGmultiplier);
   
   m_armFeedforward = new ArmFeedforward(0, m_kG, 0);

   return m_armFeedforward.calculate(Units.degreesToRadians(m_armReferencePointDEG), 0);
  }

  public void setArbFF() {
   setArmVolts(arbFFVolts());
   System.out.println("volts:"+arbFFVolts());
   System.out.println("armPosition:"+armEncoderPosition());
  }

  public void setArmVolts(double volts) {
    m_armMotor.setVoltage(volts);
  }

  public void stopArmMotor() {
    m_armMotor.disable();
  }

  public void scoreCone() {
    setArmReferenceDEG(m_armReferencePointDEG - 5);
  }

  //this runs periodically while robot is disabled
  public void updateArmSmartDashValues() {
    //updates PIDG values to what is on the smartdashboard 
    double sumValuesBefore = m_kP+m_kI+m_kD+m_kGmin+m_kGmax;

    m_kP = SmartDashboard.getNumber("Arm_kP", m_kP);
    m_kI = SmartDashboard.getNumber("Arm_kI", m_kI);
    m_kD = SmartDashboard.getNumber("Arm_kD", m_kD);
    m_kGmin = SmartDashboard.getNumber("Arm_kG", m_kGmin);
    m_kGmax = SmartDashboard.getNumber("Arm_kG", m_kGmax);

    double sumValuesAfter = m_kP+m_kI+m_kD+m_kGmin+m_kGmax;

    //checks if any values changed, if true update spark controller
    if(sumValuesBefore != sumValuesAfter) {
    //applies PID to arm spark controller 
    m_armController.setP(m_kP);
    m_armController.setI(m_kI);
    m_armController.setD(m_kD);

    System.out.println("ARM PID UPDATED!"+"kP="+m_kP+"kI="+m_kI+"kD="+m_kD+"kG_MIN="+m_kGmin+"kG_MAX="+m_kGmax);
  
    }
  }
}
