// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelescopingSubsystem extends SubsystemBase{
  private static double m_kP = 0;

  private static double m_kI = 0;

  private static double m_kD = 0;

  private final CANSparkMax m_telescopingMotor = new CANSparkMax(6, MotorType.kBrushless);

  private final RelativeEncoder m_telescopingEncoder = m_telescopingMotor.getEncoder();

  private final SparkMaxPIDController m_telescopingController = m_telescopingMotor.getPIDController();

  /** Creates a new TelescopingSubsystem. */
  public TelescopingSubsystem() {
   m_telescopingEncoder.setPositionConversionFactor(0.001778); //TODO pos conversion factor

   m_telescopingController.setP(m_kP);
   m_telescopingController.setI(m_kI);
   m_telescopingController.setD(m_kD);

   m_telescopingMotor.setInverted(false);

   m_telescopingMotor.setSoftLimit(SoftLimitDirection.kForward, (float) 12.5); //TODO set soft limits
   m_telescopingMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);

   m_telescopingMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
   m_telescopingMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
   m_telescopingMotor.burnFlash();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("telescopingEncoderPosition", m_telescopingEncoder.getPosition());
  }
}