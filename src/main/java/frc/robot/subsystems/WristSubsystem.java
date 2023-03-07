// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.wristConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class WristSubsystem extends ProfiledPIDSubsystem implements Loggable {

  private static double m_kS = 0;
  private static double m_kG = 0.65;
  private static double m_kV = 3.63;
  private static double m_kA = 0.07;

  private static double m_kP = 0;
  private static double m_kI = 0;
  private static double m_kD = 0;

  @Config
  public void setWristkS(double kS) {
    if (kS != 1) {
      m_kS = kS;
    }
  }

  @Config
  public void setWristkG(double kG) {
    if (kG != 1) {
      m_kG = kG;
    }
  }

  @Config
  public void setWristkV(double kV) {
    if (kV != 1) {
      m_kV = kV;
    }
  }

  @Config
  public void setWristkA(double kA) {
    if (kA != 0) {
      m_kA = kA;
    }
  }

  @Config
  public void setArmkP(double kP) {
    if (kP != 0) {
      m_kP = kP;
    }
  }

  @Config
  public void setArmkI(double kI) {
    if (kI != 0) {
      m_kI = kI;
    }
  }

  @Config
  public void setArmkD(double kD) {
    if (kD != 0) {
      m_kD = kD;
    }
  }

  public final CANSparkMax m_wristMotor = new CANSparkMax(wristConstants.wristID, MotorType.kBrushless);

  public final SparkMaxAbsoluteEncoder m_wristEncoder = m_wristMotor.getAbsoluteEncoder(Type.kDutyCycle);

  public ArmFeedforward m_wristFF = new ArmFeedforward(m_kS, m_kG, m_kV, m_kA);

  /** Creates a new wristSubsystem. */
  public WristSubsystem() {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            m_kP,
            m_kI,
            m_kD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(wristConstants.maxVelocityMeters, wristConstants.maxAccelMeters)));
  }

  

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    m_wristMotor.setVoltage(output + m_wristFF.calculate(setpoint.position, setpoint.velocity));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return Units.degreesToRadians(m_wristEncoder.getPosition());
  }
}
