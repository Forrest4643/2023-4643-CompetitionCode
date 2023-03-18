// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.armConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class ArmSubsystem extends PIDSubsystem implements Loggable{

  public final CANSparkMax m_armMotor = new CANSparkMax(armConstants.kArmID, MotorType.kBrushless);

  public final SparkMaxAbsoluteEncoder m_armEncoder = m_armMotor.getAbsoluteEncoder(Type.kDutyCycle);

  private Sensors m_sensors;

  private static double m_kS = 0;

  private static double m_kG = 0.65;


  private static double m_kV = 3.63;


  private static double m_kA = 0.07;


  private static double m_kP = 1;


  private static double m_kI = 0;
 

  private static double m_kD = 0;

  public ArmFeedforward armFFcontroller = new ArmFeedforward(m_kS, m_kG, m_kV, m_kA);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(Sensors m_Sensors) {
    super(
        // The ProfiledPIDController used by the subsystem
        new PIDController(
            m_kP,
            m_kI,
            m_kD
          )
    );

    this.m_sensors = m_Sensors;

  m_armEncoder.setPositionConversionFactor(360);
  m_armEncoder.setZeroOffset(118.0008674);
  m_armEncoder.setInverted(false);
  m_armMotor.setInverted(false);
  m_armMotor.getEncoder().setPositionConversionFactor(1.93359375);

  m_armMotor.getEncoder().setPosition(armEncoderPosition() - 40);

  m_armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
  m_armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

  m_armMotor.setSoftLimit(SoftLimitDirection.kForward, 80);
  m_armMotor.setSoftLimit((SoftLimitDirection.kReverse), -35);

  m_armMotor.burnFlash();



  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("armEncoderPosition", armEncoderPosition());
    SmartDashboard.putNumber("armMotorPosition", m_armMotor.getEncoder().getPosition());

    SmartDashboard.putNumber("armSetpoint", getController().getSetpoint());

  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return Units.degreesToRadians(armEncoderPosition());
  }

  public boolean atSetpoint() {
    return getController().atSetpoint();
  }

  public void unStow1() {
    getController().setSetpoint(Units.degreesToRadians(-36));
  }

  public void unStow2() {
    getController().setSetpoint(Units.degreesToRadians(-78.5));
  }

  public void matchStow() {
    getController().setSetpoint(Units.degreesToRadians(-78.5));
  }

  public void armHorizontal() {
    getController().setSetpoint(Units.degreesToRadians(0));
  }

  public double armEncoderPosition() {
    return (-m_armEncoder.getPosition() / 2.9090909090909090909090909090909);
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    m_armMotor.setVoltage(-output + 
    armFFcontroller.calculate(setpoint, 0));    
  }


}
