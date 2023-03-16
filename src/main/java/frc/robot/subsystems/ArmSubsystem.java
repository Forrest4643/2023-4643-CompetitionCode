// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.armConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class ArmSubsystem extends ProfiledPIDSubsystem implements Loggable{

  public final CANSparkMax m_armMotor = new CANSparkMax(armConstants.kArmID, MotorType.kBrushless);

  public RelativeEncoder m_armMotorEncoder = m_armMotor.getEncoder();

  public RelativeEncoder m_armEncoder = m_armMotor.getAlternateEncoder(Type.kQuadrature, 8064);

  private Sensors m_sensors;

  private static double m_kS = 0;
  @Config
  public void setArmkS(double kS) {
    if (kS != 1) {
      m_kS = kS;
    }
  }

  private static double m_kG = 0.65;
  @Config
  public void setArmkG(double kG) {
    if (kG != 1) {
      m_kG = kG;
    }
  }

  private static double m_kV = 3.63;
  @Config
  public void setArmkV(double kV) {
    if (kV != 1) {
      m_kV = kV;
    }
  }

  private static double m_kA = 0.07;
  @Config
  public void setArmkA(double kA) {
    if (kA != 0) {
      m_kA = kA;
    }
  }

  private static double m_kP = 0;
  @Config
  public void setArmkP(double kP) {
    if(kP != 0) {
      m_kP = kP;
    }
  }

  private static double m_kI = 0;
  @Config
  public void setArmkI(double kI) {
    if(kI != 0) {
      m_kI = kI;
    }
  }

  private static double m_kD = 0;
  @Config
  public void setArmkD(double kD) {
    if(kD != 0) {
      m_kD = kD;
    }
  }


  public ArmFeedforward armFFcontroller = new ArmFeedforward(m_kS, m_kG, m_kV, m_kA);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(Sensors m_Sensors) {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            m_kP,
            m_kI,
            m_kD,

            // The motion profile constraints
            new TrapezoidProfile.Constraints(
              Units.degreesToRadians(armConstants.kMaxVelocityDegSec), 
              Units.degreesToRadians(armConstants.kMaxAccelDegSec)
            )
        )
    );

    this.m_sensors = m_Sensors;

    m_armEncoder.setPositionConversionFactor(0.34375);

    m_armMotorEncoder.setPositionConversionFactor(123.75);

    m_armEncoder.setPosition(m_sensors.armNavxPitchdeg());
    m_armMotorEncoder.setPositionConversionFactor(m_sensors.armNavxPitchdeg());
  }

  @Override
  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("armMotorPosition", m_armMotorEncoder.getPosition());
    SmartDashboard.putNumber("armEncoderPosition", m_armEncoder.getPosition());
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    m_armMotor.setVoltage(output + 
      armFFcontroller.calculate(setpoint.position, setpoint.velocity)); 

  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return Units.degreesToRadians(m_armEncoder.getPosition());
  }

  public boolean atSetpoint() {
    return getController().atSetpoint();
  }

  public void unStow1() {
    getController().setGoal(36);
  }

  public void unStow2() {
    getController().setGoal(78.5);
  }

  public void matchStow() {
    getController().setGoal(78.5);
  }


}
