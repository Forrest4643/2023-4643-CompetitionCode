// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.aConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;

public class ArmSubsystem extends ProfiledPIDSubsystem implements Loggable{

  public final CANSparkMax m_armMotor = new CANSparkMax(aConstants.armID, MotorType.kBrushless);

  public RelativeEncoder m_armEncoder = m_armMotor.getEncoder();

  private Sensors m_sensors;

  private double m_kS = 0;
  @Config
  public void setArmkS(double kS) {
    if (kS != 1) {
      m_kS = kS;
    }
  }

  private double m_kG = 0.65;
  @Config
  public void setArmkG(double kG) {
    if (kG != 1) {
      m_kG = kG;
    }
  }

  private double m_kV = 3.63;
  @Config
  public void setArmkV(double kV) {
    if (kV != 1) {
      m_kV = kV;
    }
  }

  private double m_kA = 0.07;
  @Config
  public void setArmkA(double kA) {
    if (kA != 0) {
      m_kA = kA;
    }
  }


  public ArmFeedforward armFFcontroller = new ArmFeedforward(m_kS, m_kG, m_kV, m_kA);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(Sensors m_sensors) {
    super(
        // The ProfiledPIDController used by the subsystem
        new ProfiledPIDController(
            0,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(Units.degreesToRadians(10), Units.degreesToRadians(22.5))));

    this.m_sensors = m_sensors;
    m_armEncoder.setPositionConversionFactor(0.00537109374);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    m_armMotor.setVoltage(output + armFFcontroller.calculate(Units.degreesToRadians(m_sensors.armNavxPitch()), 0)); //TODO tune velocity

  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return Units.degreesToRadians(LinearFilter.movingAverage(4)
      .calculate(m_sensors.armNavxPitch()));
  }
}
